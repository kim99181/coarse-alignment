"""Read the two things this system needs out of an upstream task file.

The task file is produced by a separate planner and describes a whole job --
grasp, move, insert -- with coordinates for each step. This system deliberately
takes only two fields from it: which panel is on the bench, and which socket is
the target. Everything else, the positions above all, is ignored.

That is not laziness. The coordinates in the file are expressed in a frame the
planner detected ("detected_server_front_surface_center"), and this system
already measures the same panel itself, per frame, against its CAD. Two
independent estimates of the same quantity cannot both be used, and the one
that is re-derived from the current image is the one that stays true when the
panel is nudged. So the file is read as an *instruction* -- do this socket on
this panel -- rather than as a source of geometry.
"""
import json
import os


def _normalise(name):
    """'server_1' and 'Server 1' both key on 'server1'."""
    return ''.join(c for c in str(name).lower() if c.isalnum())


def resolve_part(server, available):
    """Map the file's server name onto a key in opening_reference.json.

    The planner writes 'server_1' where the reference table says 'server1'.
    Rather than hardcode that one substitution, both sides are reduced to
    letters and digits and matched on that, so a later 'Server-2' still lands.
    """
    if server in available:
        return server
    want = _normalise(server)
    for key in available:
        if _normalise(key) == want:
            return key
    return None


def read_task(path):
    """-> dict(part_raw, port, step_id, action, path) or raises ValueError.

    `part_raw` is the name exactly as the file gives it; call resolve_part to
    turn it into a reference-table key, which needs that table to hand.
    """
    path = os.path.expanduser(path)
    with open(path) as f:
        doc = json.load(f)

    server = doc.get('server')
    if not server:
        raise ValueError(f'{path}: no "server" field')

    # The target socket appears on more than one step -- move above it, then
    # insert into it -- and on the subtasks as well. Prefer the move, since
    # that is the step this system performs; fall back to whatever else names
    # a feature, so a file that only describes the insertion still works.
    steps = doc.get('robot_steps') or []
    picked = None
    for want in ('MOVE_ABOVE_TARGET', None):
        for step in steps:
            params = step.get('parameters') or {}
            fid = params.get('target_feature_id')
            if fid and (want is None or step.get('action') == want):
                picked = (fid, step.get('step_id'), step.get('action'))
                break
        if picked:
            break
    if picked is None:
        for sub in doc.get('converted_subtasks') or []:
            if sub.get('selected_feature_id'):
                picked = (sub['selected_feature_id'],
                          (sub.get('robot_step_ids') or [None])[0],
                          sub.get('human_action'))
                break
    if picked is None:
        raise ValueError(f'{path}: no step names a target_feature_id')

    port, step_id, action = picked
    return dict(part_raw=server, port=port, step_id=step_id, action=action,
                path=path)


def describe(task, part=None):
    """One line for the log, saying what was taken and what was ignored."""
    part_txt = f'{task["part_raw"]!r}'
    if part and part != task['part_raw']:
        part_txt += f' -> {part!r}'
    return (f'task file {os.path.basename(task["path"])}: panel {part_txt}, '
            f'target {task["port"]!r} (from {task["step_id"]} '
            f'{task["action"]}); coordinates in the file are ignored -- '
            f'position comes from vision')
