#!/usr/bin/env python3
"""Isolate why the FIRST move_j works and the SECOND does nothing.

Just established on the real device, with no homing() anywhere:

    step 4  robot.move_j(0, 0, 90, 0, 0, 0)
            before [0.57, 0.15, -0.12, ...]
            after  [-0.006, 74.919, -89.255, ...]   moved 89 deg  -> WORKED
    step 5  robot.move_j(current + 8 deg on joint_1)
            three attempts, delta 0.003 / 0.005 / 0.006 deg   -> NOTHING

So move_j is capable of driving the arm. The question is now narrow: what
differs between that first call and the one right after it? Candidates, in the
order this script tests them:

  A. TARGET EQUALS CURRENT POSE. Step 4 asked for a pose the arm was not in.
     Step 5 asked for a pose 8 deg away -- but if the device snapped to the
     commanded setpoint, a "read current, add delta" target can land on the
     device's own settled value and be rejected as no-op. Test: command an
     ABSOLUTE pose that is clearly different, and never derive it from a
     just-commanded position.

  B. SOMEONE REMEMBERS THE LAST COMMAND. If the device ignores a repeated
     vector, the second call is dropped. Test: alternate between two distinct
     absolute poses.

  C. A SETTLING / BUSY WINDOW. The second call may arrive too soon. Test: same
     target, increasing wait, three times.

  D. JOINT 1 SPECIFICALLY. Test: move joint_2 instead, and move all six.

Each trial prints the raw angle before and after, so the answer is which
condition makes the second move take effect.

SAFETY: MOVES THE ARM. Values are absolute poses a few tens of degrees apart.
Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_second_move.py
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Two clearly different absolute poses in move_j space, both near the 7 pose so
# the motion is safe: +/-20 deg on joint_1 about the known-good [0,0,90,...].
POSE_A = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
POSE_B = [20.0, 0.0, 90.0, 0.0, 0.0, 0.0]
POSE_C = [-20.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def angles(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def send(robot, target):
    try:
        robot.move_j(*target)
        return None
    except Exception as exc:  # noqa: BLE001
        return f'{type(exc).__name__}: {exc}'


def trial(robot, label, target, settle, pre=None):
    """Optionally run `pre`, then send `target`, and report the change."""
    if pre is not None:
        pre()
        time.sleep(settle)
    before = angles(robot)
    err = send(robot, target)
    time.sleep(settle)
    after = angles(robot)
    moves = [after[i] - before[i] for i in range(6)]
    biggest = max(abs(m) for m in moves)
    tag = 'MOVE' if biggest > 1.0 else 'NOOP'
    print(f'  [{tag}] {label}')
    print(f'         target {[round(v, 1) for v in target]}')
    if err:
        print(f'         raised {err}')
    else:
        print(f'         before {[round(v, 2) for v in before]}')
        print(f'         after  {[round(v, 2) for v in after]}')
        print(f'         change {[round(m, 2) for m in moves]}')
    return biggest > 1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--settle', type=float, default=3.0)
    args = ap.parse_args()

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
    print('=== probe_second_move: why does only the first move_j work? ===')
    print()
    print('No homing() is used anywhere in this script.')
    print()

    print('0. enable and go to the known-good pose')
    robot.set_enable(1)
    robot.set_rgb_mode(4)
    err = send(robot, POSE_A)
    time.sleep(args.settle)
    print(f'   move_j{POSE_A} -> {"raised " + err if err else "ok"}')
    print(f'   angles {[round(v, 2) for v in angles(robot)]}')
    print()
    if err:
        print('FATAL: the first move_j failed; cannot continue.')
        return 1

    print('A. absolute poses, never derived from a fresh read')
    print('   (tests whether a target equal to the settled pose is dropped)')
    trial(robot, 'A1: go to POSE_B (joint_1 = +20)', POSE_B, args.settle)
    trial(robot, 'A2: back to POSE_A', POSE_A, args.settle)
    trial(robot, 'A3: go to POSE_C (joint_1 = -20)', POSE_C, args.settle)
    trial(robot, 'A4: back to POSE_A again', POSE_A, args.settle)
    print()

    print('B. the same target twice in a row')
    print('   (tests whether a repeated vector is ignored)')
    trial(robot, 'B1: POSE_B', POSE_B, args.settle)
    trial(robot, 'B2: POSE_B again (identical)', POSE_B, args.settle)
    print()

    print('C. increasing settle time before re-commanding')
    for wait in (1.0, 3.0, 8.0):
        trial(robot, f'C: POSE_C after a {wait:.0f}s wait', POSE_C, wait)
    print()

    print('D. other axes')
    trial(robot, 'D1: joint_2 = +20', [0.0, 20.0, 90.0, 0.0, 0.0, 0.0],
          args.settle)
    trial(robot, 'D2: joint_3 = 60 (from 90)', [0.0, 0.0, 60.0, 0.0, 0.0, 0.0],
          args.settle)
    trial(robot, 'D3: all six shifted',
          [10.0, 10.0, 100.0, 10.0, 10.0, 10.0], args.settle)
    print()

    print('=' * 62)
    print('Interpret the pattern above:')
    print('  - If A/B/C mostly move, then the earlier failure was only that')
    print('    the target happened to equal the settled pose, and the driver')
    print('    is FINE: it sends absolute poses from trajectories.')
    print('  - If only the very first call after enable moves and everything')
    print('    afterwards is dead, the device accepts one setpoint per session')
    print('    and that is a firmware behaviour to work around.')
    print('  - If D moves but A/B/C do not, joint_1 specifically is the issue.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
