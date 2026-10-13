#!/usr/bin/env python3
"""Replicate the reference driver's exact startup, with no homing() at all.

Everything tried so far shares one property: homing() ran before move_j.

    probe_move     : read -> set_enable -> move_j        NOOP
    probe_interface: set_enable -> move_j, set_position  NOOP
    probe_enable   : homing -> setup -> move_j           NOOP (every setup)
    probe_mode     : homing -> set_command_mode(0..5)
                     -> move_j                           NOOP (every mode)

The reference driver that demonstrably works on this same arm does this:

    my_driver = ref_tool.find_any()
    my_driver.robot.set_enable(1)
    my_driver.robot.set_rgb_mode(4)
    self.move_rad(self.ready_rad)        # move_j([0, 0, 90, 0, 0, 0])

It never calls homing(). move_j is its FIRST motion command, issued
immediately after set_enable. That is the one combination never tested here,
and homing() is the prime suspect for switching the device into a state where
setpoint commands are ignored -- which would explain why homing keeps working
while everything else does nothing.

This script reproduces the reference sequence literally, reading angles after
each step. If move_j works here, the driver must stop calling homing() on
startup.

SAFETY: this MOVES THE ARM. Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_reference_flow.py
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# The reference driver's ready_rad after rad_fix: (0+[0,0,1.57079,0,0,0])*1
# in RADIANS, converted to degrees -> [0, 0, 90, 0, 0, 0].
READY_MOVEJ = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def angles(robot):
    return [round(float(getattr(robot, n).angle), 3) for n in JR]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--settle', type=float, default=3.0)
    ap.add_argument('--nudge', type=float, default=8.0,
                    help='degrees to then nudge joint_1 by')
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
    print('=== probe_reference_flow: the reference driver\'s sequence, '
          'no homing() ===')
    print()

    print('1. angles on arrival (device state before we touch anything)')
    print(f'   {angles(robot)}')
    print()

    # ---- step 2: set_enable(1), exactly as the reference does -----------
    print('2. robot.set_enable(1)   <-- int 1, matching the reference')
    try:
        robot.set_enable(1)
        print('   ok')
    except Exception as exc:  # noqa: BLE001
        print(f'   raised {type(exc).__name__}: {exc}')
        print('   (the reference uses 1; a bool may be treated differently)')
    print(f'   angles after  {angles(robot)}')
    print()

    # ---- step 3: set_rgb_mode(4), as the reference does -----------------
    print('3. robot.set_rgb_mode(4)   (reference does this before moving)')
    try:
        robot.set_rgb_mode(4)
        print('   ok')
    except Exception as exc:  # noqa: BLE001
        print(f'   raised {type(exc).__name__}: {exc}')
    print()

    # ---- step 4: move_j IMMEDIATELY, with no homing ---------------------
    print('4. robot.move_j(0, 0, 90, 0, 0, 0)   <-- first motion command,')
    print('   issued immediately after set_enable, exactly like the reference')
    before = angles(robot)
    try:
        robot.move_j(*READY_MOVEJ)
        print('   move_j returned ok')
    except Exception as exc:  # noqa: BLE001
        print(f'   raised {type(exc).__name__}: {exc}')
    time.sleep(args.settle)
    after = angles(robot)
    print(f'   before {before}')
    print(f'   after  {after}')
    moved = max(abs(after[i] - before[i]) for i in range(6))
    print(f'   max |delta| = {moved:.3f} deg')
    if moved > 1.0:
        print('   [MOVE] move_j DOES work in the reference sequence')
    else:
        print('   [NOOP] still nothing')
    print()

    # ---- step 5: a nudge, to confirm it is a live control path ----------
    print(f'5. nudge joint_1 by {args.nudge:+.1f} deg via move_j')
    base = [float(getattr(robot, n).angle) for n in JR]
    target = list(base)
    target[0] = base[0] + args.nudge
    for attempt in range(1, 4):
        try:
            robot.move_j(*target)
        except Exception as exc:  # noqa: BLE001
            print(f'   attempt {attempt} raised {type(exc).__name__}: {exc}')
            break
        time.sleep(args.settle)
        now = [float(getattr(robot, n).angle) for n in JR]
        delta = now[0] - base[0]
        print(f'   attempt {attempt}: joint_1 {base[0]:.3f} -> {now[0]:.3f} '
              f'(delta {delta:+.3f})')
        if abs(delta) > 1.0:
            print('   [MOVE] the setpoint path is live after this sequence')
            break
    else:
        print('   [NOOP] three attempts, no movement')
    print()

    print('=' * 62)
    print('If step 4 or 5 shows [MOVE], then homing() is what breaks move_j,')
    print('and the fix is simply to stop calling homing() on startup -- the')
    print('reference driver never does. That would also explain why homing')
    print('keeps working while nothing else does.')
    print()
    print('If step 4 and 5 are both [NOOP], then even the reference sequence')
    print('fails on this unit today, which points at the controller state or')
    print('firmware rather than the command sequence.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
