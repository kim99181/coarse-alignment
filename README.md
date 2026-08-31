# tm_ws — vision-guided connector alignment

A TM5-900 arm, a Toyo CHG2 gripper and an Intel RealSense D405 on the wrist,
positioning the gripper over a named socket on a multi-port panel.

Scope is alignment only. Getting the plug the last few millimetres into the
socket is done separately, under admittance control.

## What is here

Only `src/py_gripper` is tracked here -- pose estimation, the arm command line
and the CAD tooling. It needs two third-party packages dropped alongside it in
`src/`, neither of which belongs in this repository:

| | |
|---|---|
| `src/tm2_ros2` | Techman's ROS 2 driver: <https://github.com/TechmanRobotInc/tm2_ros2> (branch `humble`) |
| `src/robotic` | Robotiq 85 gripper packages |

```bash
git clone -b humble https://github.com/TechmanRobotInc/tm2_ros2.git src/tm2_ros2
```

## How the pose is found

One greyscale frame and the CAD, per frame, at about 27 Hz:

1. **Find the panel** — Otsu over the whole frame. It is black on a pale bench,
   which is the easiest threshold in the pipeline. The largest *solid* blob
   wins, not the largest one; cables are darker still but fill a fraction of
   their bounding box.
2. **Find the ports** — adaptive threshold inside the panel only, with the
   neighbourhood sized from the working distance. Deliberately over-detects.
3. **Decide which port is which** — two blobs paired with two CAD ports fix a
   similarity outright, so the search enumerates those pairings and scores each
   by how many other ports land where it predicts. The mapping is forced to be
   mirrored, because the port face points at the camera.
4. **Solve the pose** — `SOLVEPNP_SQPNP` against the CAD port centres. The
   distance comes out of this solve; it is not measured by the depth sensor.
5. **Refine** — re-measure each port inside its own projected outline and solve
   again.

The depth stream is used for one thing: a RANSAC plane fit that pins the panel's
tilt, which a single planar view is genuinely weak at. Everything else --
position, scale, heading, which port is which -- is image and CAD only. On a
matte black panel that is the point: its depth returns cannot carry a scale.

Measured on hardware: 8 of 8 ports matched, 0.75 mm mean reprojection.

## Running it

```bash
colcon build --packages-select py_gripper     # from the workspace root
source install/setup.bash
```

Vision, arm driver and camera:

```bash
export ROS_LOCALHOST_ONLY=1
ros2 launch py_gripper tmr_depth_launch.py part:=server1
```

Arm command line, in a second terminal:

```bash
ros2 run py_gripper arm_cmd --ros-args -p part:=server1
```

`ROS_LOCALHOST_ONLY=1` is not optional on a shared lab network -- another
machine on the default domain will otherwise publish into the same topics.

### Settings that do not travel

Three things in `config/` and one in the launch file are specific to the rig
they were measured on. On the same robot they carry over; on a different one
they do not, and using them unchanged will put the arm in the wrong place:

| | |
|---|---|
| `config/ICA_Lab_UMI_Config.yaml` | hand-eye calibration -- where the camera sits on the flange, including its 24.4 deg tilt |
| `config/opening_reference.json` | port table, valid only for the panel it was generated from |
| `robot_ip` in `launch/tmr_depth_launch.py` | |
| `table_z`, `roll_usb`, `roll_hdmi`, `roll_rj45` on `arm_cmd` | bench height, and how each plug sits in the jaws |

### Driving it from a task file

A planner upstream emits a task file naming the panel and the socket. One is
committed at `src/py_gripper/tasks/robot_script.json` -- point both nodes at it
and the job runs itself:

```bash
ros2 launch py_gripper tmr_depth_launch.py task_json:=src/py_gripper/tasks/robot_script.json
ros2 run py_gripper arm_cmd --ros-args -p task_json:=src/py_gripper/tasks/robot_script.json
```

Run those from the workspace root, or give an absolute path. To target a
different socket, edit `target_feature_id` in that file, or name a port on the
command line -- `go rj451` overrides it for one move without touching the file.

`arm_cmd` then squares up over the target and descends `auto_drop_m` (20 cm by
default) as a single move, once vision has a lock. Only two fields are read --
the panel and the target socket. The coordinates in the file are ignored on
purpose: this system measures the same panel per frame against its CAD, and the
estimate re-derived from the current image is the one that stays true when the
panel is nudged.

Without a task file nothing runs automatically and every command below behaves
as it always has. Pass `part:=server1` in that case, so the right CAD table is
loaded.

| parameter | |
|---|---|
| `auto_run` | set false to load the task but not act on it |
| `auto_drop_m` | how far to descend on that first move (default 0.20) |
| `auto_delay_s` | pause before it, 0 by default; set 1-2 while trying something new |

### Commands

| | |
|---|---|
| `go [port] [roll] [20cm] [dry]` | square up over the target **and descend**, one move |
| `holexy [port] [roll] [20cm] [dry]` | square up over a port; descends only if given a distance |
| `hole [port] [roll] [5cm] [dry]` | square up, and stand off from the port **face** |
| `h [5cm\|5mm] [dry]` | straight down, pose untouched |
| `ready [45cm] [dry]` | park above the bench, flange level |
| `xy` | move over the panel centre, height unchanged |
| `task` / `task <path>` | show the loaded job, or load another |
| `x y z` / `x y z rx ry rz` | move to a pose directly |
| `1` / `0` | gripper open / close |

Arguments are recognised by shape rather than position, so they can be given in
any order: a bare word is a port name, a bare number is a roll angle, anything
suffixed `cm` or `mm` is a distance, and `dry` prints the target without moving.
`go usb2 90 15cm dry` reads the way it looks.

With a task file loaded, `go`, `holexy` and `hole` use its target when no port
is named.

Note that `cm` means two different things, deliberately. On `hole` it is an
absolute height above the port face; on `go`, `holexy` and `h` it is how far to
descend from wherever the arm is now. The relative ones report what will be
left underneath before they move.

`dry` is worth using every time. The distances are all measured to the
`tool_pose` the driver reports, so whatever is held in the jaws hangs below the
number shown.

Port names come from the CAD table: `usb1`-`usb4`, `rj451`, `rj452`,
`hdmi1`, `hdmi2`. A debug stream runs at <http://localhost:8091/>, showing the
detection and the raw camera frame side by side -- the raw one because a frame
that produced no pose carries almost no annotation, which is exactly when you
need to see what the detector was given.

### Running a whole cycle

`arm_cmd` does one socket and stops. A job is several cables, and this system
performs only one of the things each cable needs -- the insertion and the next
grasp belong to other programs -- so `arm_cmd_cycle` is mostly a loop of
waiting:

| | |
|---|---|
| align + descend | this program |
| pause | insertion program drives the plug home and opens the jaws |
| back to the start pose | this program |
| pause | grasp program fetches the next cable |

It subclasses `ArmCmd`, so finding a socket, matching it against the CAD and
turning to it are the same code; `arm_cmd` itself is untouched and still behaves
exactly as described above. Two cycles are committed:

| | |
|---|---|
| `tasks/robot_script_cycle.json` | `usb1 -> rj452 -> hdmi1` |
| `tasks/robot_script_cycle2.json` | `rj451 -> rj452 -> usb3 -> hdmi2` |

A run begins with a cable already in the jaws, so set the arm up by hand first.
This program never touches the gripper, where `arm_cmd` opens it on startup --
which is the point, since opening it would drop the cable before the first move.

```bash
ros2 run py_gripper arm_cmd                      # no task file
```

`ready`, put the cable in the jaws, `0` to close, then place the arm where the
camera sees the whole panel. Ctrl-C, and:

```bash
ros2 run py_gripper arm_cmd_cycle --ros-args \
    -p task_json:=src/py_gripper/tasks/robot_script_cycle2.json
```

That pose is recorded at the start of the run and returned to between sockets,
as a plain move to remembered numbers with no camera in the loop. An earlier
version asked vision to re-centre instead and that stalled runs after the first
socket: from 25 cm up and off to one side the panel is not recognisable enough
to aim with, and refusing to aim with a stale pose means refusing to continue.
Going back to one pose also keeps every socket measured from the same view,
rather than each one further off-axis than the last.

| parameter | |
|---|---|
| `insert_pause_s` | hand-off to the insertion program (default 3) |
| `regrasp_pause_s` | hand-off to the grasp program (default 5) |
| `sequence` | `"usb1,rj452,hdmi1"`, overriding the task file |
| `home_keep_heading` | carry the wrist's heading home instead of restoring the recorded one |
| `max_heading_swing_deg` | refuse a target that would turn the wrist further (default 150) |
| `lift_first` | retract straight up before travelling home |
| `settle_s`, `fresh_frames` | how long to stand still, and how many new panel poses to wait for, before trusting one |

Two commands are added to the prompt -- `seq [ports...] [20cm] [dry]` to run a
cycle by hand, and `home` / `home set` to return to the recorded start pose or
record a new one. Everything in the table above still works, `seq dry` included,
which walks the whole run printing each target without moving.

#### Joint 6, and why the order matters

Every socket on this panel shares one long axis, so what sets a socket's heading
is its kind: usb and hdmi take a quarter turn, rj45 takes none, leaving the two
90 deg apart. Grouping sockets by kind therefore costs the wrist less --
`cycle2` turns it once in the whole run, where `cycle` alternates kinds and
turns twice.

It also makes `xy` a poor way to start a cycle. Its heading is hardcoded to
rz = -90 deg, which is half a turn from an rj45's, and the controller reports
such a target as out of range. Place the arm by hand instead. A turn past
`max_heading_swing_deg` is refused with an explanation rather than left to fault
mid-move, and the turn each move needs is printed either way.

## Adding a panel

Models live in `src/py_gripper/cad/`. Drop the STL there, add it to
`tools/build_reference.py`'s `MESHES`, then:

```bash
~/.local/bin/micromamba run -n foundationpose python tools/build_reference.py
```

It ray-casts the port face, measures each opening and writes
`config/opening_reference.json` -- the table the runtime actually reads. The
models for the panels here are committed, so a fresh clone can regenerate that
table rather than having to trust the copy in the repository.

One thing worth knowing when designing the panel: make the port layout
asymmetric under **reflection and a half turn**, not just under rotation. The
panel here is very nearly symmetric under both, and two separate classes of
mis-identification came out of that.

## Tests

```bash
python3 tools/test_mono.py
```

Runs the pipeline over two captured frames -- the panel in two orientations,
different lighting -- and checks the port labelling against the physical
hardware. Two frames rather than one on purpose: every regression this pipeline
has had was invisible in the first.
