#!/usr/bin/env python3
"""Settle whether a SINGLE-joint command works, using literal values only.

The previous probe concluded that move_j is ignored when only one joint has to
travel. The user pushed back, correctly: during the joint_sign work they
commanded individual joints and those moved. So the conclusion is probably
wrong, and the real difference is somewhere else.

Re-reading the evidence, the cleanest split is not "how many joints must move"
but "literal value vs value built from a readback":

    probe_shape 1  [0.0,   0.0,  90.0,  0.0,    0.0,  0.0]  literal   MOVED
    probe_shape 2  [0.0,   0.0,  90.0,  0.0,    0.0,  0.0]  literal   NOOP
    probe_shape 3  [20.0,  0.0,  90.0,  0.0,    0.0,  0.0]  literal   MOVED
    probe_shape 5  [19.99, 75.0, -90.0, -0.001, -0.0, -0.001] derived NOOP
    probe_shape 6  [20.0,  0.0,  90.0,  0.0,    0.0,  0.0]  literal   MOVED

Note step 2 is a literal and still did nothing -- because the arm was already
exactly there. So "literal" alone does not decide it either. The one thing that
does is whether the target differs from the current pose in a way the device
accepts.

This script therefore keeps the command LITERAL and varies only WHICH JOINTS
TRAVEL, which is the thing the user actually did during sign testing:

    1. multi-joint literal          [0, 0, 90, 0, 0, 0] from an offset pose
    2. SINGLE-joint literal         [20, 0, 90, 0, 0, 0]   (only joint_1 moves)
    3. SINGLE-joint literal, j2     [0, 20, 90, 0, 0, 0]
    4. SINGLE-joint literal, j3     [0, 0, 60, 0, 0, 0]
    5. two-joint literal            [20, 0, 60, 0, 0, 0]

Every command is a fixed literal; nothing is built from a readback. Each trial
is preceded by an explicit literal return to the base pose and a wait, so the
starting state is identical every time.

If the single-joint literals move, then single-joint control is fine, the
earlier conclusion was wrong, and the real problem is confined to how the
DRIVER builds its targets -- which is the far more useful answer.

SAFETY: MOVES THE ARM, within about 30 deg of the proven-good pose. Keep a hand
on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_single.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

BASE = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def raw(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def go(robot, cmd, wait):
    try:
        robot.move_j(*[float(v) for v in cmd])
    except Exception as exc:  # noqa: BLE001
        return f'{type(exc).__name__}: {exc}'
    time.sleep(wait)
    return None


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
    wait = 4.0
    print('=== probe_single: single-joint control with LITERAL values ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)
    print(f'on arrival   {[round(v, 3) for v in raw(robot)]}')

    # Get to base from wherever we are, using a literal.
    err = go(robot, BASE, wait)
    print(f'go to BASE   {[round(v, 3) for v in raw(robot)]}'
          f'{" (raised " + err + ")" if err else ""}')
    print()

    trials = [
        ('1. multi-joint literal: j1->20, j3->60',
         [20.0, 0.0, 60.0, 0.0, 0.0, 0.0]),
        ('2. SINGLE joint literal: only j1 -> 20',
         [20.0, 0.0, 90.0, 0.0, 0.0, 0.0]),
        ('3. SINGLE joint literal: only j2 -> 20',
         [0.0, 20.0, 90.0, 0.0, 0.0, 0.0]),
        ('4. SINGLE joint literal: only j3 -> 60',
         [0.0, 0.0, 60.0, 0.0, 0.0, 0.0]),
        ('5. SINGLE joint literal: only j1 -> -20',
         [-20.0, 0.0, 90.0, 0.0, 0.0, 0.0]),
        ('6. multi-joint literal again, to confirm liveness',
         [20.0, 0.0, 60.0, 0.0, 0.0, 0.0]),
    ]

    for label, target in trials:
        # Always start from BASE, commanded literally and waited out.
        go(robot, BASE, wait)
        before = raw(robot)
        need = [abs(target[i] - before[i]) for i in range(6)]
        n_need = sum(1 for d in need if d > 1.0)

        print(f'--- {label}')
        print(f'    before   {[round(v, 2) for v in before]}')
        print(f'    sending  {target}')
        print(f'    joints needing to travel: {n_need}')
        err = go(robot, target, wait)
        after = raw(robot)
        moved = [after[i] - before[i] for i in range(6)]
        n_moved = sum(1 for d in moved if abs(d) > 1.0)
        verdict = 'MOVED' if n_moved else 'NOOP '
        print(f'    after    {[round(v, 2) for v in after]}')
        print(f'    change   {[round(d, 2) for d in moved]}')
        print(f'    [{verdict}] {n_moved} joint(s) moved'
              f'{"  (raised " + err + ")" if err else ""}')
        print()

    print('=' * 62)
    print('If trials 2-5 move (single-joint literals), then single-joint')
    print('control works and the earlier "one joint" conclusion was wrong.')
    print('The remaining difference between these and the driver would then be')
    print('purely how the target values are produced, which is a much smaller')
    print('and more fixable problem.')
    print()
    print('If trials 2-5 do NOT move while 1 and 6 do, then the device really')
    print('does refuse a command in which only one joint travels, and the')
    print('driver must batch its moves differently.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
