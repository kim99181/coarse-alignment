"""Read a whole run out of an upstream task file, not just its first socket.

task_file.read_task answers "what is the job", singular. That is what a single
insertion needs, and what the vision node wants for its debug overlay, and it
is left exactly as it was. A file that lists several sockets describes a cycle,
and this module returns all of them, in the order the planner wrote them.

That order is the one thing here that cannot be re-derived from the image.
Which socket is which gets measured every frame against the CAD; which one goes
first is a decision already made upstream, and the only way to know it is to
read it.

Everything else about how the file is read is unchanged on purpose. The panel
name is resolved by the same rules -- resolve_part is imported rather than
copied, so the two modules cannot drift apart on it -- and the coordinates in
the file are ignored here for exactly the reasons the original module sets out.
"""
import json
import os

# Imported, not duplicated: 'server_1' -> 'server1' is one rule and there
# should be one copy of it.
from py_gripper.task_file import resolve_part      # noqa: F401  (re-exported)


def read_sequence(path):
    """-> dict(part_raw, targets=[dict(port, step_id, action), ...], path).

    Raises ValueError if the file names no socket at all, which is the one
    failure worth being loud about: a cycle over nothing is not a cycle.

    Consecutive repeats collapse into one visit. A socket normally appears
    twice in these files, once to move above it and once to insert into it,
    and counting both would send the arm back to a socket it just left.
    """
    path = os.path.expanduser(path)
    with open(path) as f:
        doc = json.load(f)

    server = doc.get('server')
    if not server:
        raise ValueError(f'{path}: no "server" field')

    # Every step that names a socket, in file order. When the file separates
    # "move above" from "insert", keep only the moves -- that is the step this
    # system performs, and the insert steps belong to the program this one
    # hands over to. A file describing insertions alone still works: then there
    # are no moves to prefer and every naming step counts.
    named = [(step.get('step_id'), step.get('action'),
              (step.get('parameters') or {}).get('target_feature_id'))
             for step in doc.get('robot_steps') or []]
    named = [n for n in named if n[2]]
    picked = [n for n in named if n[1] == 'MOVE_ABOVE_TARGET'] or named

    if not picked:
        picked = [((sub.get('robot_step_ids') or [None])[0],
                   sub.get('human_action'), sub['selected_feature_id'])
                  for sub in doc.get('converted_subtasks') or []
                  if sub.get('selected_feature_id')]
    if not picked:
        raise ValueError(f'{path}: no step names a target_feature_id')

    targets = []
    for step_id, action, port in picked:
        if targets and targets[-1]['port'] == port:
            continue
        targets.append(dict(port=port, step_id=step_id, action=action))
    return dict(part_raw=server, targets=targets, path=path)


def describe_sequence(seq, part=None):
    """One line for the log, saying what was taken and what was ignored."""
    part_txt = f'{seq["part_raw"]!r}'
    if part and part != seq['part_raw']:
        part_txt += f' -> {part!r}'
    ports = ' -> '.join(t['port'] for t in seq['targets'])
    return (f'task file {os.path.basename(seq["path"])}: panel {part_txt}, '
            f'{len(seq["targets"])} target(s): {ports}; coordinates in the '
            f'file are ignored -- position comes from vision')
