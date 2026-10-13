#!/usr/bin/env python3
"""Measure the per-joint offset between the command space and the readback.

The previous version of this probe was wrong, and the data showed it: it read
the angles 2.5 s after commanding a 75/90 deg move and got [0.276, 68.436,
-80.803, ...], then read again later in the same run and got [0, 75, -90, ...].
Same command, different positions -- the first read caught the arm mid-travel.
Every offset it printed was therefore worthless, and the per-joint push trials
that followed compared against a stale base.

This version fixes both mistakes:

  * it WAITS FOR SETTLE by polling until successive reads agree, instead of
    sleeping a fixed time and hoping;
  * every trial returns to an explicitly commanded absolute pose, never to "the
    last thing we read";
  * it prints the command it sends alongside the readback, so the offset is
    visible rather than inferred.

The offset is the thing that broke the last fix: joint 3's readback sits about
180 deg from its move_j value, which puts the nearest-whole-turn rounding
boundary exactly at the midpoint, so round() picks the wrong turn and the
driver sends -268.53 for a +20 deg intent.

SAFETY: MOVES THE ARM. Deltas are small (default 5 deg on one joint at a time).
Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_gain.py
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# A pose already proven to work on this unit.
BASE_CMD = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def raw(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def wait_settle(robot, timeout=12.0, quiet=0.6, eps=0.05):
    """Poll until the readback stops changing.

    A fixed sleep is what broke the previous version: a 75 deg joint move takes
    longer than the settle time used, so the "after" reading was taken while
    the arm was still travelling.
    """
    t0 = time.monotonic()
    last = raw(robot)
    stable_since = time.monotonic()
    while time.monotonic() - t0 < timeout:
        time.sleep(0.15)
        now = raw(robot)
        if max(abs(now[i] - last[i]) for i in range(6)) > eps:
            stable_since = time.monotonic()
        elif time.monotonic() - stable_since > quiet:
            return now
        last = now
    return raw(robot)


def go_to(robot, cmd, label):
    """Command an absolute pose and wait until it actually settles."""
    cmd = [float(v) for v in cmd]
    try:
        robot.move_j(*cmd)
    except Exception as exc:  # noqa: BLE001
        print(f'    {label}: move_j raised {type(exc).__name__}: {exc}')
        return None
    got = wait_settle(robot)
    print(f'    {label}: sent {[round(v, 2) for v in cmd]}')
    print(f'    {label}: read {[round(v, 2) for v in got]}')
    return got


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delta', type=float, default=5.0)
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
    print('=== probe_gain: command space vs readback space, per joint ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)

    print('1. drive to the known-good pose and WAIT FOR SETTLE')
    base = go_to(robot, BASE_CMD, 'base')
    if base is None:
        return 1
    print()

    print('   per-joint offset = raw readback - move_j command')
    print(f'   {"joint":<9} {"move_j":>10} {"readback":>10} {"offset":>10}')
    offsets = []
    for i in range(6):
        off = base[i] - BASE_CMD[i]
        offsets.append(off)
        flag = ''
        if abs(abs(off) - 180.0) < 25.0:
            flag = '   <-- near half a turn: rounding will be ambiguous'
        print(f'   joint_{i + 1:<3} {BASE_CMD[i]:>10.3f} {base[i]:>10.3f} '
              f'{off:>10.3f}{flag}')
    print()
    print('   Feed these into the calibration. Joint 3 is the one that put the')
    print('   nearest-whole-turn rounding exactly at the midpoint.')
    print()

    print(f'2. push each joint by {args.delta:+.1f} from the settled base')
    print(f'   {"joint":<9} {"before":>10} {"after":>10} {"achieved":>10} '
          f'{"ratio":>8}')
    for i in range(6):
        # Always return to the absolute base pose, and wait for it, so this
        # trial starts from a known place rather than from "wherever we are".
        if go_to(robot, BASE_CMD, f'  reset for joint_{i + 1}') is None:
            continue
        before = raw(robot)
        cmd = list(before)
        cmd[i] = before[i] + args.delta
        try:
            robot.move_j(*cmd)
        except Exception as exc:  # noqa: BLE001
            print(f'   joint_{i + 1}: raised {type(exc).__name__}: {exc}')
            continue
        after = wait_settle(robot)
        achieved = after[i] - before[i]
        ratio = achieved / args.delta if abs(args.delta) > 1e-9 else float('nan')
        print(f'   joint_{i + 1:<3} {before[i]:>10.3f} {after[i]:>10.3f} '
              f'{achieved:>+10.3f} {ratio:>8.3f}')
    print()

    print('=' * 62)
    print('ratio ~ +1  the joint follows the command one to one')
    print('ratio ~ -1  it moves the other way (sign error)')
    print('ratio ~ 0   it ignored the command')
    print()
    worst = max(range(6), key=lambda i: abs(abs(offsets[i]) - 180.0))
    if abs(abs(offsets[worst]) - 180.0) < 25.0:
        print(f'joint_{worst + 1} has an offset of {offsets[worst]:+.1f} deg,')
        print('i.e. its readback sits about half a turn from its command value.')
        print('That is what makes the nearest-whole-turn conversion ambiguous,')
        print('so the fix must pin this joint using the measured offset rather')
        print('than assuming the two spaces differ by whole turns only.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
