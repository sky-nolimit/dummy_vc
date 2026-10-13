#!/usr/bin/env python3
"""Measure how the device actually responds to move_j on each joint.

Why: the nearest-equivalent conversion assumed the device's raw angle readback
and its move_j command space are the same space modulo whole turns. For joint 3
they are not -- homing() reports raw j3 = -90 while move_j j3 = 90, a 180 deg
discrepancy, and the measured 7 pose reports -91.474 for the same command. That
puts the rounding boundary exactly at the midpoint, so round() picks the wrong
turn and the driver sends -268.53.

Rather than keep reasoning about it, this measures the response directly:

    for each joint, at several deltas:
        read raw
        move_j(current + delta)         <- delta applied to ONE joint only
        read raw
        report the achieved change

If the device follows the command, achieved ~= delta, and we get the real gain.
If a joint moves by a different amount, or not at all, or in the opposite
direction, that is recorded per joint instead of assumed.

It also reports whether the raw readback for that joint is offset from the
move_j value by a fixed amount, which is what the calibration needs to know.

SAFETY: MOVES THE ARM. Deltas are small (a few degrees per joint). Keep a hand
on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_gain.py
    python3 tools/probe_gain.py --delta 5
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# Where homing() leaves the arm, expressed the way homing reports it.
HOME_RAW = [0.0, 75.0, -90.0, 0.0, 0.0, 0.0]


def raw(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delta', type=float, default=5.0,
                    help='degrees to push each joint by (default 5)')
    ap.add_argument('--settle', type=float, default=2.5)
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
    print('=== probe_gain: what each joint actually does when commanded ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)

    # Establish a known base pose with a command we know works, then treat the
    # raw readback there as the base for every trial.
    print('going to the pose move_j = [0, 0, 90, 0, 0, 0] ...')
    robot.move_j(0.0, 0.0, 90.0, 0.0, 0.0, 0.0)
    time.sleep(args.settle)
    base = raw(robot)
    print(f'  raw readback there = {[round(v, 3) for v in base]}')
    print()
    print('This is the key table. The move_j values we sent were')
    print('  [0, 0, 90, 0, 0, 0]')
    print('so any difference between that and the readback above is the')
    print('per-joint offset between the command space and the readback space.')
    print()
    print(f'  {"joint":<8} {"move_j sent":>12} {"raw readback":>13} '
          f'{"raw - move_j":>13}')
    sent = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
    for i in range(6):
        print(f'  joint_{i + 1:<2} {sent[i]:>12.3f} {base[i]:>13.3f} '
              f'{base[i] - sent[i]:>13.3f}')
    print()

    # Now push each joint by a known delta and see what the readback does.
    print(f'=== pushing each joint by {args.delta:+.1f} via move_j ===')
    print(f'  {"joint":<8} {"raw before":>11} {"raw after":>11} '
          f'{"achieved":>10} {"ratio":>8}')
    results = {}
    for i in range(6):
        # Return to base first so trials do not accumulate.
        try:
            robot.move_j(*[base[j] + (0.0) for j in range(6)])
        except Exception:  # noqa: BLE001
            pass
        time.sleep(1.0)

        before = raw(robot)
        cmd = list(before)
        cmd[i] = before[i] + args.delta
        try:
            robot.move_j(*cmd)
        except Exception as exc:  # noqa: BLE001
            print(f'  joint_{i + 1:<2} raised {type(exc).__name__}: {exc}')
            continue
        time.sleep(args.settle)
        after = raw(robot)
        achieved = after[i] - before[i]
        ratio = achieved / args.delta if abs(args.delta) > 1e-9 else float('nan')
        results[i] = (achieved, ratio)
        print(f'  joint_{i + 1:<2} {before[i]:>11.3f} {after[i]:>11.3f} '
              f'{achieved:>+10.3f} {ratio:>8.3f}')

    print()
    print('=' * 62)
    print('How to read it:')
    print('  ratio ~ 1.0   the joint follows the command one to one')
    print('  ratio ~ -1.0  the joint moves the OPPOSITE way (sign error)')
    print('  ratio ~ 0.0   the joint ignored the command')
    print('  other ratios  the command and readback spaces differ by a scale')
    print()
    print('The "raw - move_j" column is the per-joint offset the calibration')
    print('must reproduce. If joint 3 shows about -180 there, that is the')
    print('half-turn mismatch that made the nearest-equivalent conversion pick')
    print('the wrong turn and send -268.53.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
