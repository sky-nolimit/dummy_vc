#!/usr/bin/env python3
"""Compare two command SHAPES, because that is where the evidence points.

What actually happened on the hardware:

    probe_second_move   fixed absolute poses A/B/C alternating   all MOVED
    probe_wrap          fixed absolute poses                     MOVED
    probe_gain step 1   fixed absolute pose [0,0,90,0,0,0]       MOVED (the user
                                                                 saw the arm go
                                                                 to the base pose)
    probe_gain step 2   target = readback + 5 on one joint       all ratio 0

So the arm moves for a command written as a literal absolute pose, and ignores
one built from the just-read angles. This script tests exactly that, holding
everything else constant: for the SAME physical destination, send it once as a
fixed literal and once as readback-plus-delta, and print the angles for both.

If the literal moves and the derived one does not, the target values must
differ, and printing them side by side will show how.

SAFETY: MOVES THE ARM, up to about 20 deg on joint 1. Keep a hand on the power
switch.

    pkill -f dummy_controller
    python3 tools/probe_shape.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

HOME = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
# The same physical destination written as a literal.
LIT = [20.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def raw(robot):
    return [round(float(getattr(robot, n).angle), 3) for n in JR]


def send(robot, cmd, label, wait):
    print(f'--- {label}')
    print(f'    before       {raw(robot)}')
    cmd = [round(float(v), 3) for v in cmd]
    print(f'    sending      {cmd}')
    try:
        robot.move_j(*cmd)
        print('    move_j ok')
    except Exception as exc:  # noqa: BLE001
        print(f'    move_j RAISED {type(exc).__name__}: {exc}')
        return None
    time.sleep(wait)
    after = raw(robot)
    print(f'    after        {after}')
    moved = max(abs(after[i] - cmd[i]) for i in range(6)) if False else None
    print()
    return after


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
    print('=== probe_shape: literal absolute pose vs readback-plus-delta ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)
    print(f'on arrival   {raw(robot)}')
    print()

    # 1. get to home with a literal
    send(robot, HOME, '1. literal HOME [0,0,90,0,0,0]', wait)
    cur = raw(robot)

    # 2. the SAME destination, but built by reading first and adding
    derived = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
    send(robot, derived, '2. derived, same destination '
                         '(literal + 0, i.e. identical values)', wait)

    # 3. literal +20 on joint 1
    send(robot, LIT, '3. literal joint_1 = 20', wait)

    # 4. back to home, literal
    send(robot, HOME, '4. literal HOME again', wait)

    # 5. read the pose, then add +20 and send THAT
    before = [float(getattr(robot, n).angle) for n in JR]
    cmd = list(before)
    cmd[0] = before[0] + 20.0
    print(f'    (readback used as the base: '
          f'{[round(v, 3) for v in before]})')
    send(robot, cmd, '5. readback + 20 on joint_1', wait)

    # 6. literal +20 again, to prove the arm can still move at all
    send(robot, LIT, '6. literal joint_1 = 20, to test liveness', wait)

    print('=' * 62)
    print('Compare steps 3 and 5: they ask for the same physical place.')
    print('  - If 3 moves and 5 does not, the two command vectors differ;')
    print('    look at the printed values to see how.')
    print('  - If 6 (a literal, right after 5) also fails, then the device')
    print('    stops responding after a derived command, or needs a longer')
    print('    gap between commands.')
    print('  - If everything moves, the earlier zeros were my measurement')
    print('    logic and the driver is fine.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
