#!/usr/bin/env python3
"""Test whether move_j is ignored when only ONE joint needs to travel.

The two commands that did nothing, and the ones that worked, differ in how many
joints had somewhere to go:

    move_j [0.0,   0.0,  90.0,  0.0,    0.0,  0.0]   arm was already at HOME,
                                                     0 joints needed to move  -> NOOP
    move_j [19.99, 75.0, -90.0, -0.001, -0.0, -0.001]  only joint_1 had to go
                                                     ~20 deg, the other 5 were
                                                     already there            -> NOOP
    move_j [20.0,  0.0,  90.0,  0.0,    0.0,  0.0]  joints 2 and 3 had to go
                                                     75 and 90 deg            -> MOVED
    move_j [0.0,   0.0,  90.0,  0.0,    0.0,  0.0]  joints 2 and 3 had to go
                                                     75 and 90 deg back       -> MOVED

That is not "literal vs derived" -- both NOOP cases were derived, but so is the
real driver command. The distinguishing factor looks like the NUMBER OF JOINTS
THAT MUST MOVE.

This matters because the driver's failing command is exactly the single-joint
case: deg=[-20.22, 0.43, -268.53, 0.91, 0.04, 0.04] against a device sitting at
[0.215, 74.572, -91.474, -0.91, -0.035, -0.040] -- only joint_1 (and joint_3,
whose value is the suspect -268.53) have anywhere to go.

Each trial starts from a freshly commanded pose and reports the raw angles
before and after, so the number of joints that moved is visible directly.

SAFETY: MOVES THE ARM. Angles stay within about 20 deg of the proven-good pose.
Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_oneshot.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

HOME = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def raw(robot):
    return [round(float(getattr(robot, n).angle), 3) for n in JR]


def trial(robot, label, cmd, wait=4.0):
    """Send cmd from the current pose and report which joints moved."""
    before = [float(getattr(robot, n).angle) for n in JR]
    cmd = [float(v) for v in cmd]
    need = [abs(cmd[i] - before[i]) for i in range(6)]
    n_need = sum(1 for d in need if d > 1.0)
    print(f'--- {label}')
    print(f'    before    {[round(v, 3) for v in before]}')
    print(f'    sending   {[round(v, 3) for v in cmd]}')
    print(f'    joints that need to move: {n_need}  '
          f'(deltas {[round(d, 2) for d in need]})')
    try:
        robot.move_j(*cmd)
        print('    move_j ok')
    except Exception as exc:  # noqa: BLE001
        print(f'    move_j RAISED {type(exc).__name__}: {exc}')
        print()
        return None
    time.sleep(wait)
    after = [float(getattr(robot, n).angle) for n in JR]
    moved = [after[i] - before[i] for i in range(6)]
    n_moved = sum(1 for d in moved if abs(d) > 1.0)
    verdict = 'MOVED' if n_moved else 'NOOP '
    print(f'    after     {[round(v, 3) for v in after]}')
    print(f'    change    {[round(d, 2) for d in moved]}')
    print(f'    [{verdict}] {n_moved} joint(s) actually moved')
    print()
    return n_moved


def main():
    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except ImportError as exc:
        print(f'FATAL: cannot import dummy_cli_tool ({exc})')
        return 2

    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first.')
        return 1
    robot = dev.robot
    print('=== probe_oneshot: does move_j need MORE THAN ONE joint to travel? ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)
    print(f'on arrival   {raw(robot)}')
    print()

    # Trial 1: get to HOME. Joints 2 and 3 have far to go, so this should work.
    # The readback afterwards is what the later trials start from.
    trial(robot, 'A. go to HOME (j2, j3 must travel) -> expected to WORK',
          HOME)
    base = raw(robot)
    print(f'    settled at {base}')
    print()

    # Trial 2: SINGLE joint moves, from the settled pose. Keep every other
    # value at exactly what we just read, so each of these has exactly one
    # joint with somewhere to go.
    for idx in (0, 1, 2):
        cmd = [float(getattr(robot, n).angle) for n in JR]
        cmd[idx] = cmd[idx] + 15.0
        trial(robot, f'B{idx + 1}. move ONLY joint_{idx + 1} by +15 '
                     f'-> hypothesis: ignored', cmd)
        # return to the settled base before the next single-joint trial
        trial(robot, f'   reset toward base for the next trial', HOME)

    # Trial 3: move joint_1 AND leave joints 2/3 somewhere different, so three
    # joints have to travel. If this works, the count is what matters.
    trial(robot, 'C. joint_1 to 20 while j2/j3 also differ '
                 '(3 joints travel) -> expected to WORK',
          [20.0, 20.0, 70.0, 0.0, 0.0, 0.0])

    print('=' * 62)
    print('If A and C move but every B is ignored, then the device refuses a')
    print('move_j in which only one joint has anywhere to go. The driver sends')
    print('exactly that shape for a single-joint test, which is why its')
    print('trajectories appear to do nothing.')
    print()
    print('That would also mean the previous "shape" theory was wrong: the')
    print('NOOP cases are simply the ones where fewer joints had to travel.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
