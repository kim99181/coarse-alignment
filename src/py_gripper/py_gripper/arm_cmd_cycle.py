"""The alignment half of a multi-socket run, as a loop.

arm_cmd does one socket: square up over it, descend, stop. That is the right
shape for dialling an alignment in, and the wrong shape for a job, because a
job is several cables and this system performs only one of the things each
cable needs. The rest belongs to other programs, so the loop here is mostly
waiting:

    align + descend            <- this file
    pause                         insertion program drives the plug home and
                                  opens the jaws
    back to the start pose     <- this file
    pause                         grasp program fetches the next cable
    ... and again for the next socket

The way back is one move: straight to the pose the run started from, the one a
person set up by hand at ready height, squared over the panel with the whole of
it in view. The controller interpolates, so the arm rises and travels at the
same time rather than stopping level on the way.

That assumes the jaws are clear of the socket when control comes back. They
should be -- the insertion program opens them and retracts -- but if a plug
ever gets dragged out of its socket on the way home, lift_first puts a vertical
retract back in front of the travel.

Going back there matters more than it looks. This system re-measures the panel
from scratch on every frame, so each socket's target is only as good as the
view it was measured from. Left alone, the arm works its way across the panel
and measures each socket from further off-axis than the last, which is exactly
how the second socket in a run comes out worse than the first even though the
panel never moved.

And the return is a move to remembered numbers, with no camera in the loop.
An earlier version asked vision to re-centre instead and that is what stalled a
run after the first socket: from 25cm up and off to one side the panel was not
recognisable enough to aim with, and refusing to aim with a stale pose meant
refusing to continue. Remembered numbers have no such failure mode -- and
because the arm ends up exactly where the last good measurement was taken, a
pose left over from before is not stale at all. The panel has not moved; the
camera is back where it was.

Everything about how a socket is found, matched and turned to is inherited from
ArmCmd unchanged. This file adds the loop, the waiting, and one guard against
reading a panel pose that was measured while the camera was moving.
"""
import json
import os
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from py_gripper import task_file_cycle
from py_gripper.arm_cmd import ArmCmd, parse_hole_args


class ArmCmdCycle(ArmCmd):
    """ArmCmd plus a loop over several sockets. Nothing else is changed."""

    def __init__(self):
        super().__init__()
        # Counters the loop uses to tell "a command went out" and "a new
        # measurement arrived" from "nothing happened". Both are maintained by
        # the overrides below; seed them here in case neither has fired yet.
        self._pose_seq = getattr(self, '_pose_seq', 0)
        self._sent = getattr(self, '_sent', 0)
        self._fb_seq = getattr(self, '_fb_seq', 0)
        # Where the run started: ready height, squared over the panel, set up
        # by hand before this program was launched. Recorded once at the top of
        # run_cycle and returned to between sockets.
        self.home_pose = None

        # The sockets to visit, when they are not coming from the task file.
        # Commas or spaces: "usb1,rj452,hdmi1". Useful for rehearsing an order
        # the file does not describe without editing the file.
        self.declare_parameter('sequence', '')
        # The two hand-offs, in seconds. This system neither inserts nor
        # grasps; at each socket it holds the arm still while another program
        # does one of those. Sleeping stands in for that hand-off -- enough to
        # rehearse the order and the motions, and the place a real handshake
        # goes when there is one to talk to.
        self.declare_parameter('insert_pause_s', 3.0)
        self.declare_parameter('regrasp_pause_s', 5.0)
        # Carry the wrist's heading home rather than restoring the one the
        # start pose was recorded with. Position still goes back exactly.
        #
        # This is about joint 6, not about the camera. Every socket on this
        # panel shares one long axis, so what sets a socket's heading is its
        # kind: usb and hdmi take a quarter turn, rj45 takes none, leaving the
        # two 90 deg apart. Restoring a fixed heading at home puts a swing
        # either side of every socket, and if that fixed heading happens to sit
        # half a turn from one of them -- which is exactly where align_xy's
        # hardcoded rz of -90 deg lands relative to an rj45 -- the wrist is
        # asked for 180 deg and the controller reports the target as out of
        # range. Carrying the heading instead means the wrist only ever turns
        # between one socket and the next: 90 deg here, never 180.
        #
        # Vision does not mind. The panel is found by matching its port pattern,
        # which carries no preferred image orientation, and the arm's own pose
        # goes into the world-frame transform either way.
        self.declare_parameter('home_keep_heading', True)
        # Refuse a target that would swing the wrist further than this. A run
        # stopped with a message is easier to work with than a controller fault
        # mid-move, and a swing this large means something is wrong rather than
        # merely awkward -- most likely the panel's heading has been estimated
        # half a turn out, which flips every socket's long axis at once. Set it
        # to 360 to allow anything.
        self.declare_parameter('max_heading_swing_deg', 150.0)
        # Retract straight up before travelling home, rather than going there
        # in one interpolated move. Off: the insertion program opens the jaws
        # and retracts before handing back, so there is nothing to clear. Turn
        # it on if a plug is ever dragged out of its socket on the way home.
        self.declare_parameter('lift_first', False)
        # Height of that retract above the table. Only lift_first uses it --
        # the run itself returns to the recorded start pose, whatever height
        # that was set at.
        self.declare_parameter('ready_height_m', 0.45)
        # Whether to go back to the pose the run started from between sockets.
        # On, for the reason in the module docstring. Off makes the run measure
        # each socket from wherever the previous one left the camera.
        self.declare_parameter('return_home', True)
        # Additionally ask vision to centre on the panel after that. Off: the
        # start pose is already centred, having been set up by hand over the
        # panel, so this only adds a step that can fail. Kept because a run
        # that starts somewhere else might want it.
        self.declare_parameter('recentre', False)
        # How long to stand still before trusting a panel pose, and how many
        # whole new estimates to wait for after that. See settle_and_refresh.
        self.declare_parameter('settle_s', 0.5)
        self.declare_parameter('fresh_frames', 5)
        self.declare_parameter('arrive_timeout_s', 30.0)

    # ------------------------------------------------------------------ hooks

    def _load_task(self, path):
        """Take the whole list of sockets, not just the first.

        Called from ArmCmd.__init__, so it must leave task_targets defined on
        every path through -- including the one where there is no file.
        """
        self.task_targets = []
        if not path:
            return
        try:
            task = task_file_cycle.read_sequence(path)
        except (OSError, ValueError, json.JSONDecodeError) as e:
            self.get_logger().error(f'cannot read task file: {e}')
            return
        refs_path = os.path.join(get_package_share_directory('py_gripper'),
                                 'config', 'opening_reference.json')
        try:
            refs = json.load(open(refs_path))
        except OSError as e:
            self.get_logger().error(f'cannot read {refs_path}: {e}')
            return
        part = task_file_cycle.resolve_part(task['part_raw'], refs)
        if part is None:
            self.get_logger().error(
                f'task file names panel {task["part_raw"]!r}, which is not in '
                f'{refs_path} (have {sorted(refs)}) -- keeping {self.part!r}')
            return
        self.part = part
        self._ports = None                       # force a reload for this panel
        self.task_targets = [t['port'] for t in task['targets']]
        self.get_logger().info(task_file_cycle.describe_sequence(task, part))

        names = {p['name'] for p in refs[part].get('ports', [])}
        if names:
            unknown = [p for p in self.task_targets if p not in names]
            if unknown:
                self.get_logger().error(
                    f'target(s) {unknown} are not ports on {part!r} '
                    f'(have {sorted(names)}) -- dropping them')
                self.task_targets = [p for p in self.task_targets
                                     if p in names]
        # The one-shot commands inherited from ArmCmd read task_port, so keep
        # it pointing at the first socket: `holexy` with no name still means
        # "the socket the job starts with".
        self.task_port = self.task_targets[0] if self.task_targets else None

    def object_pose_callback(self, msg):
        super().object_pose_callback(msg)
        self._pose_seq = getattr(self, '_pose_seq', 0) + 1

    def pos_callback(self, msg):
        super().pos_callback(msg)
        self._fb_seq = getattr(self, '_fb_seq', 0) + 1

    def send_request(self, *args, **kwargs):
        self._sent = getattr(self, '_sent', 0) + 1
        return super().send_request(*args, **kwargs)

    def _sent_a_move(self, call):
        """Run one of ArmCmd's motion commands; report whether it sent anything.

        Those commands all return the service future's result, which rclpy
        hands back as None until the call completes -- so None means nothing at
        all, and a refusal is indistinguishable from a success. Rather than
        change their return values, which would mean editing the file this one
        is deliberately not touching, watch whether send_request was reached.
        """
        n = self._sent
        call()
        return self._sent > n

    # ------------------------------------------------------------- the pieces

    def _sequence_ports(self):
        """The sockets to visit, in order: the parameter first, else the file."""
        raw = str(self.get_parameter('sequence').value or '')
        ports = [p for p in raw.replace(',', ' ').split() if p]
        if not ports:
            ports = list(self.task_targets)
        table = self.port_table()
        if table:
            unknown = [p for p in ports if p not in table]
            if unknown:
                self.get_logger().error(
                    f'unknown port(s) {unknown} on {self.part!r}; have '
                    f'{sorted(table)} -- dropping them from the sequence')
                ports = [p for p in ports if p in table]
        return ports

    def _pause(self, seconds, why):
        """Hold still while another program works.

        The hand-off, and for now only the shape of one. What matters while
        rehearsing is that the wait is visible: a silent gap looks exactly like
        a hang, so it counts down. Swapping the sleep for a real handshake
        later changes nothing else about the loop.
        """
        if seconds <= 0:
            return
        self.get_logger().info(f'--- pausing {seconds:.0f}s: {why} ---')
        t0 = time.time()
        tick = seconds - 1.0
        while True:
            left = seconds - (time.time() - t0)
            if left <= 0:
                break
            if tick >= 1.0 and left <= tick:
                self.get_logger().info(f'    {left:.0f}s left')
                tick -= 1.0
            time.sleep(0.05)
        self.get_logger().info('--- resuming ---')

    def settle_and_refresh(self, why=''):
        """Stop, then wait for panel poses measured after stopping. -> bool.

        The vision node multiplies the arm's *current* flange pose by a pose
        solved from a camera frame that is already tens of milliseconds old.
        Standing still that mismatch is nothing; moving at 0.1 m/s it is
        millimetres, and the result is wrong in a way nothing downstream can
        detect -- the pose looks as valid as any other.

        wait_until_arrived returns as soon as the tool is inside a 5mm ball of
        its target, which is not the same as stopped, so the newest estimate at
        that moment was very likely measured in motion. So: stand still, then
        throw away everything already in hand and wait for whole new frames.

        Returns False if the new frames never came, which means vision has lost
        the panel rather than that the arm is unsteady. The caller decides what
        to do about it; this reports rather than blocks forever.
        """
        settle = float(self.get_parameter('settle_s').value)
        frames = int(self.get_parameter('fresh_frames').value)
        if settle > 0:
            time.sleep(settle)
        if frames <= 0:
            return True
        start = self._pose_seq
        t0 = time.time()
        while self._pose_seq - start < frames:
            if time.time() - t0 > 5.0:
                self.get_logger().warn(
                    f'only {self._pose_seq - start} new panel pose(s) in 5s'
                    + (f' ({why})' if why else '')
                    + ' -- vision has probably lost the panel')
                return False
            time.sleep(0.02)
        return True

    def _heading_change(self, R_target):
        """Signed turn about vertical from the current flange X to R_target's.

        Measured as an angle between two horizontal directions rather than as a
        difference of euler rz values, because euler xyz reports a downward
        flange as rx near +pi or near -pi interchangeably and the rz that comes
        with each differs by half a turn. The vectors have no such ambiguity.
        """
        R_cur = Rotation.from_euler('xyz', self.current_positions[3:6]).as_matrix()
        a, b = R_cur[:, 0].copy(), np.asarray(R_target)[:, 0].copy()
        a[2] = b[2] = 0.0
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-6 or nb < 1e-6:
            return 0.0                    # flange X is vertical: no heading
        a, b = a / na, b / nb
        return float(np.degrees(np.arctan2(float(np.cross(a, b)[2]),
                                           float(np.dot(a, b)))))

    def _send_hole_target(self, target_pos, R_world_flange, label, move):
        """ArmCmd's version, with the wrist turn reported and bounded.

        Every socket-aligned move goes through here, so this is the one place
        that sees what the wrist is about to be asked for. ArmCmd already logs
        the total reorientation as a magnitude; what matters for joint 6 is the
        turn about vertical, and its sign, which is what gets printed.
        """
        turn = self._heading_change(R_world_flange)
        limit = float(self.get_parameter('max_heading_swing_deg').value)
        self.get_logger().info(f'  wrist turn for this move: {turn:+.0f}deg')
        if abs(turn) > limit:
            self.get_logger().error(
                f'{label}: this would swing the wrist {turn:+.0f}deg, past the '
                f'{limit:.0f}deg limit. Refusing before the controller reports '
                f'the target out of range. Either the panel heading has been '
                f'estimated half a turn out -- check the labels on the debug '
                f'stream -- or the run started from a heading too far from the '
                f'sockets; raise max_heading_swing_deg to allow it anyway.')
            return False
        return super()._send_hole_target(target_pos, R_world_flange, label, move)

    def _wait_for_feedback(self, timeout=5.0):
        """Block until tm_driver has reported a real tool_pose at least once.

        current_positions starts life as a placeholder, and a placeholder
        recorded as the start pose would send the arm to a made-up point later
        in the run. Wait for the real thing rather than trusting the default.
        """
        t0 = time.time()
        while getattr(self, '_fb_seq', 0) == 0:
            if time.time() - t0 > timeout:
                self.get_logger().error(
                    'no feedback_states from tm_driver -- the start pose would '
                    'be a placeholder rather than where the arm actually is; '
                    'not moving')
                return False
            time.sleep(0.05)
        return True

    def capture_home(self):
        """Remember where the arm is now as the pose to come back to."""
        self.home_pose = [float(v) for v in self.current_positions[:6]]
        self.get_logger().info(
            f'start pose recorded: {[round(v, 4) for v in self.home_pose]} '
            f'({self.clearance_note(self.home_pose[2])})')
        return self.home_pose

    def go_home(self, move=True):
        """Return to the pose the run started from. -> bool.

        No vision involved, which is the whole point -- see the module
        docstring. Position and orientation both, so the camera ends up looking
        at the panel from exactly the angle the run was set up at.
        """
        if self.home_pose is None:
            self.get_logger().error(
                'no start pose recorded -- run the cycle, or use "home set" '
                'to record where the arm is now')
            return False
        target = list(self.home_pose)
        keep = bool(self.get_parameter('home_keep_heading').value)
        if keep:
            # Position back exactly, orientation left where the socket put it.
            target[3:6] = [float(v) for v in self.current_positions[3:6]]
        if not self.target_is_sane(target):
            return False
        self.get_logger().info(
            f'back to the start pose -> {[round(v, 4) for v in target]} '
            f'({self.clearance_note(target[2])}, '
            f'{"keeping the current heading" if keep else "restoring the recorded heading"})')
        if not move:
            self.get_logger().info('(dry run, not moving)')
            return True
        self.send_request(target)
        return True

    def recentre_over_panel(self, move=True):
        """Put the camera back over the middle of the panel. -> bool.

        align_xy keeps the current height and only changes X/Y, so this is a
        sideways move at ready height with nothing underneath it.
        """
        if not move:
            self.get_logger().info('re-centring over the panel (dry run)')
            return True
        if not self.settle_and_refresh('before re-centring'):
            self.get_logger().error(
                'no fresh panel pose to centre on -- is the panel in view at '
                'this height? stopping rather than aiming with a stale one')
            return False
        if not self._sent_a_move(self.align_xy):
            self.get_logger().error('re-centring refused; stopping')
            return False
        self.wait_until_arrived(
            timeout=float(self.get_parameter('arrive_timeout_s').value))
        return True

    # ----------------------------------------------------------------- the run

    def run_cycle(self, ports=None, drop=None, roll_deg=0.0, move=True,
                  height=None):
        """Work through the sockets in order. -> True if all of them were done.

        Each socket is: go, wait, come back, wait. Only the moving parts are
        this file's; both waits belong to other programs.

        Going and coming back are one move each. The controller interpolates to
        a single pose, so the alignment happens with the descent and the climb
        happens with the travel; stopping level in between would only add a
        pause. The last socket comes back like all the others and then stops,
        skipping only the final wait -- that one exists to set up the next
        cable, and after the last there is no next. So a finished run leaves
        the arm exactly where it started, ready to go again.

        The pose the run starts from is recorded here, on the first line that
        moves anything, because that is when the arm is standing where a person
        put it.

        Stops at the first socket it cannot compute a target for, rather than
        carrying on. A run that quietly skipped a step would finish looking
        exactly like one that did everything it was asked.
        """
        if ports is None:
            ports = self._sequence_ports()
        if not ports:
            self.get_logger().warn(
                'no sequence to run -- give a task file, set the "sequence" '
                'parameter, or name them: seq usb1 rj452 hdmi1')
            return False
        if drop is None:
            drop = float(self.get_parameter('auto_drop_m').value)
        if height is None:
            height = float(self.get_parameter('ready_height_m').value)
        insert_pause = float(self.get_parameter('insert_pause_s').value)
        regrasp_pause = float(self.get_parameter('regrasp_pause_s').value)
        return_home = bool(self.get_parameter('return_home').value)
        lift_first = bool(self.get_parameter('lift_first').value)
        recentre = bool(self.get_parameter('recentre').value)
        arrive_timeout = float(self.get_parameter('arrive_timeout_s').value)

        if move and not self.wait_for_vision():
            self.get_logger().error(
                'no panel pose after 20s -- is the vision node running and '
                'does the debug stream show the ports? not moving')
            return False

        n = len(ports)
        self.get_logger().info(
            f'cycle on {self.part!r}: {" -> ".join(ports)}')
        self.get_logger().info(
            f'  at each socket: align + descend {drop*100:.0f}cm as one move, '
            f'{insert_pause:.0f}s for the insertion'
            + (f', straight up to {height*100:.0f}cm above the bench'
               if lift_first else '')
            + (', back to the start pose in one move' if return_home
               else f', up to {height*100:.0f}cm above the bench')
            + (', re-centre on the panel' if recentre else '')
            + f', {regrasp_pause:.0f}s for the next cable')

        try:
            if move and not self._wait_for_feedback():
                return False
            # Recorded on a dry run too, so the printed plan includes the
            # return legs rather than stopping at the first socket.
            self.capture_home()
            if move and not self.settle_and_refresh('before the first socket'):
                self.get_logger().error(
                    'vision has no lock on the panel -- not starting')
                return False

            for i, port in enumerate(ports, 1):
                self.get_logger().info(
                    f'=== [{i}/{n}] {port}: aligning and descending ===')
                if not self._sent_a_move(
                        lambda: self.align_hole_xy(roll_deg=roll_deg, move=move,
                                                   port=port, drop=drop)) \
                        and move:
                    self.get_logger().error(
                        f'[{i}/{n}] {port}: no target -- stopping here rather '
                        f'than skipping to the next socket')
                    return False
                if move:
                    self.wait_until_arrived(timeout=arrive_timeout)
                    self._pause(insert_pause,
                                f'insertion program works on {port}, and opens '
                                f'the jaws')

                # Optional vertical retract, for the case where something is
                # still in the way of a diagonal move.
                if lift_first or not return_home:
                    self.get_logger().info(
                        f'=== [{i}/{n}] {port}: straight up to ready ===')
                    if not self._sent_a_move(
                            lambda: self.ready_pose(height=height, move=move)) \
                            and move:
                        self.get_logger().error('ready pose refused; stopping')
                        return False
                    if move:
                        self.wait_until_arrived(timeout=arrive_timeout)

                if return_home:
                    self.get_logger().info(
                        f'=== [{i}/{n}] back to the start pose ===')
                    if not self.go_home(move=move):
                        return False
                    if move:
                        self.wait_until_arrived(timeout=arrive_timeout)

                if i >= n:
                    break              # last socket done, and already parked
                if recentre:
                    self.get_logger().info(
                        f'=== [{i}/{n}] re-centring on the panel ===')
                    if not self.recentre_over_panel(move=move):
                        return False
                if move:
                    # One more refresh now the arm is back at the overhead
                    # view, so the next socket's target is measured from there
                    # rather than from somewhere on the way.
                    #
                    # A failure here is not fatal any more. The arm is standing
                    # exactly where the last good measurement was taken and the
                    # panel has not moved, so the estimate already in hand is
                    # the right one -- just older. Say so and carry on.
                    if not self.settle_and_refresh('before the next socket'):
                        self.get_logger().warn(
                            'carrying on with the panel pose already in hand: '
                            'the arm is back at the pose that pose was measured '
                            'from, and the panel has not moved')
                    self._pause(regrasp_pause,
                                f'grasp program fetches the cable for '
                                f'{ports[i]}')

            self.get_logger().info(
                f'cycle finished: {n} socket(s) done, parked at '
                + ('the pose the run started from' if return_home
                   else f'ready ({height*100:.0f}cm above the bench)'))
            return True
        except KeyboardInterrupt:
            # Ctrl-C stops this thread; the arm carries on to the pose it was
            # last sent unless something says otherwise.
            self.get_logger().warn('cycle interrupted -- sending STOP')
            self.send_event()
            return False


def parse_cycle_args(args):
    """Arguments after seq -> (port names, roll, dry, drop).

    Unlike the one-socket commands this takes several names, so the tokens are
    read one at a time through the same parser and collected.
    """
    names, roll, dry, drop = [], 0.0, False, None
    for a in args:
        port, r, d, standoff = parse_hole_args([a])
        if d:
            dry = True
        elif standoff is not None:
            drop = standoff
        elif port is not None:
            names.append(port)
        else:
            roll = r
    return names, roll, dry, drop


def main(args=None):
    rclpy.init(args=args)
    node = ArmCmdCycle()
    rclpy.spin_once(node)

    # Deliberately does not touch the jaws. A cycle starts with a cable already
    # gripped by the program that hands over to this one, and arm_cmd's habit
    # of opening them on startup would drop it before the first move. To open
    # them here, type "1" at the prompt.

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    ports = node._sequence_ports()
    if ports and node.get_parameter('auto_run').value:
        delay = float(node.get_parameter('auto_delay_s').value)
        if delay > 0:
            node.get_logger().info(
                f'starting in {delay:.0f}s -- Ctrl-C now to stop')
            time.sleep(delay)
        node.run_cycle(ports=ports)

    while True:
        raw = input("Positions: ").strip()
        head = raw.split()[0] if raw.split() else ''
        # "seq" / "seq usb1 rj452 hdmi1" / "seq 15cm" / "seq dry"
        if head in ('seq', 'cycle'):
            names, roll, dry, drop = parse_cycle_args(raw.split()[1:])
            node.run_cycle(ports=names or None, roll_deg=roll,
                           move=not dry, drop=drop)
            continue
        # "home" goes back to the recorded start pose; "home set" records
        # where the arm is now as that pose. Useful for putting the arm back
        # after an interrupted run without re-running the whole cycle.
        if head == 'home':
            rest = raw.split()[1:]
            if rest and rest[0] in ('set', 's'):
                node.capture_home()
            else:
                node.go_home(move=not (rest and rest[0] in ('dry', 'd')))
            continue
        # Everything below is arm_cmd's own prompt, unchanged, so a run can be
        # driven by hand when the loop is not what is wanted.
        if head in ('h', 'hover'):
            _p, _roll, dry, drop = parse_hole_args(raw.split()[1:])
            kwargs = {} if drop is None else {'drop': drop}
            node.descend(move=not dry, **kwargs)
            continue
        if raw in ('xy', 'align'):
            node.align_xy()
            continue
        if head in ('go', 'run'):
            _p, roll, dry, drop = parse_hole_args(raw.split()[1:])
            node.run_task(drop=drop, roll_deg=roll, move=not dry)
            continue
        if head == 'task':
            arg = raw.split(maxsplit=1)
            if len(arg) > 1:
                node._load_task(arg[1].strip('"\''))
            else:
                seq = node._sequence_ports()
                node.get_logger().info(
                    f'panel {node.part!r}, sequence '
                    f'{" -> ".join(seq) if seq else "(none -- name one per command)"}')
            continue
        if head in ('ready', 'rdy'):
            _p, _roll, dry, height = parse_hole_args(raw.split()[1:])
            kwargs = {} if height is None else {'height': height}
            node.ready_pose(move=not dry, **kwargs)
            continue
        if head in ('hole', 'a'):
            port, roll, dry, standoff = parse_hole_args(raw.split()[1:])
            kwargs = {} if standoff is None else {'standoff': standoff}
            node.align_to_hole(roll_deg=roll, move=not dry, port=port, **kwargs)
            continue
        if head in ('holexy', 'hxy'):
            port, roll, dry, drop = parse_hole_args(raw.split()[1:])
            node.align_hole_xy(roll_deg=roll, move=not dry, port=port,
                               drop=drop or 0.0)
            continue
        positions = list(map(float, raw.split()))
        if len(positions) == 3:
            positions = positions + [3.14159, 0.0, 3.14]
        if len(positions) == 1:
            node.send_gripper(0.085 if positions[0] != 0 else 0.0)
            continue
        if len(positions) == 0:
            node.send_event()
            continue
        node.get_logger().info("Response: %s" % node.send_request(positions))

    rclpy.shutdown()


if __name__ == '__main__':
    main()
