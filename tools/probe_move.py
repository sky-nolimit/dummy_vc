#!/usr/bin/env python3
"""Ask the device directly whether it accepts a motion command.

Why this exists: the driver log showed

    waypoint 1/1 deg=[339.78, 0.43, 91.47, ...]
    Publish JPOS: j1: 0.00, ...

so the correct value reached robot.move_j() with no error, and the arm did not
move. That places the fault below the driver. This script removes everything
else -- no ROS, no node, no trajectory, no timers -- and drives the device
directly, reading the raw angles before and after each attempt. Whichever step
first shows no change is where the chain breaks.

It must run WITHOUT dummy_controller, because only one process can hold the
fibre link at a time.

    # stop the driver first (Ctrl+C in its terminal), then:
    python3 tools/probe_move.py
    python3 tools/probe_move.py --joint 1 --delta 10
    python3 tools/probe_move.py --mode set_position

Read the output as a checklist: the first FAIL is the answer.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


def read_angles(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def show(label, vals):
    print(f'    {label:<26} {[round(v, 3) for v in vals]}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--joint', type=int, default=1, help='1..6')
    ap.add_argument('--delta', type=float, default=10.0,
                    help='degrees to add to the current angle readback')
    ap.add_argument('--mode', default='move_j',
                    choices=['move_j', 'set_position'])
    ap.add_argument('--timeout', type=float, default=15.0)
    ap.add_argument('--settle', type=float, default=3.0)
    args = ap.parse_args()

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except ImportError as exc:
        print(f'FATAL: cannot import dummy_cli_tool ({exc}).')
        print('Run this on the Linux side after `source install/setup.bash`.')
        return 2

    print('=== probe_move: drive the device with no ROS in the loop ===')
    print()
    print('1. connect')
    dev = ref_tool.find_any(path='usb', timeout=args.timeout)
    if dev is None:
        print('   [FAIL] no device found.')
        print('          Is dummy_controller still running? Only one process')
        print('          can hold the fibre link. Check with:')
        print('            ps aux | grep -E "dummy|ros2 launch" | grep -v grep')
        return 1
    robot = dev.robot
    print('   [ok  ] connected')

    print()
    print('2. read angles BEFORE enabling (is the link healthy at all?)')
    try:
        before = read_angles(robot)
        show('raw angle (deg)', before)
        print('   [ok  ] reads work, so the fibre link is alive')
    except Exception as exc:  # noqa: BLE001
        print(f'   [FAIL] reading angles raised: {type(exc).__name__}: {exc}')
        print('          The link is up but the device is not answering.')
        print('          Try a power cycle of the arm.')
        return 1

    print()
    print('3. enable motors')
    try:
        robot.set_enable(True)
        print('   [ok  ] set_enable(True) returned')
    except Exception as exc:  # noqa: BLE001
        print(f'   [FAIL] set_enable raised: {type(exc).__name__}: {exc}')
        return 1

    print()
    print('4. send the motion command')
    idx = args.joint - 1
    target = list(before)
    target[idx] = before[idx] + args.delta
    kind = 'move_j(absolute)' if args.mode == 'move_j' else 'set_position'
    print(f'   mode   : {kind}')
    print(f'   joint  : joint_{args.joint}')
    print(f'   from   : {before[idx]:.3f}')
    print(f'   to     : {target[idx]:.3f}')
    try:
        if args.mode == 'move_j':
            # move_j takes ALL six joints; keep the others where they are.
            robot.move_j(target[0], target[1], target[2],
                         target[3], target[4], target[5])
        else:
            getattr(robot, JR[idx]).set_position(target[idx])
        print('   [ok  ] call returned without raising')
    except Exception as exc:  # noqa: BLE001
        print(f'   [FAIL] command raised: {type(exc).__name__}: {exc}')
        return 1

    print()
    print(f'5. wait {args.settle}s, then read angles AFTER')
    time.sleep(args.settle)
    try:
        after = read_angles(robot)
        show('raw angle (deg)', after)
    except Exception as exc:  # noqa: BLE001
        print(f'   [FAIL] readback raised: {type(exc).__name__}: {exc}')
        return 1

    moved = [abs(after[i] - before[i]) for i in range(6)]
    print()
    print('   per-joint change (deg):')
    for i in range(6):
        mark = '  <-- commanded' if i == idx else ''
        print(f'     joint_{i+1}: {after[i] - before[i]:+9.3f}{mark}')
    print()

    print('=' * 62)
    if moved[idx] < 0.5:
        print('RESULT: FAIL -- the device ACCEPTED the command and did NOT')
        print('        move. This is NOT a pass: motion is not happening.')
        print()
        print('  The command path is fine, so the fault is in the device or')
        print('  its power stage, not in this project\'s code. Things to check:')
        print('    - power cycle the arm, then rerun this probe')
        print('    - is the arm mechanically blocked or the brake engaged?')
        print('    - did the gripper fault earlier put the controller into a')
        print('      fault/limp state? (a power cycle clears that)')
        print('    - does the vendor tool move the arm? If not, it is hardware')
        return 1
    print(f'RESULT: joint_{args.joint} moved {moved[idx]:.3f} deg '
          '(commanded %.3f).' % abs(args.delta))
    print('  The device responds to direct commands, so the fault is above')
    print('  this level: back to the driver / trajectory path.')
    other = [i for i in range(6) if i != idx and moved[i] > 0.5]
    if other:
        print(f'  NOTE: joints {[i+1 for i in other]} also moved, which they '
              'should not have. That points at the calibration or the joint '
              'ordering rather than the transport.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
