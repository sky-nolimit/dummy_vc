#!/usr/bin/env python3
"""Find the value range move_j accepts, per joint.

The user's observation, confirmed by the j3 sweep:

    j3 =   90      accepted
    j3 =   91.474  accepted
    j3 =  -270      refused   <- mathematically the same angle
    j3 =  -268.526  refused   <- what the driver used to send
    j3 =  450       refused
    j3 =    0       refused
    j3 =  -90       refused
    j3 =  270       refused

So the device does NOT fold whole turns together. It has a hard accepted range,
and values outside it are refused SILENTLY -- the whole command is dropped,
which is why the arm does not move at all.

## Timing (this is why the script waits instead of sleeping)

An earlier version slept a fixed 0.8 s after the return-to-base command and a
fixed 2.5 s after the test command. **That was wrong and it is the same mistake
that produced a set of all-zero results in probe_gain.** The arm is slow: a
75-90 deg move can take longer than 2.5 s, so the "after" reading was taken
mid-travel and every sample was junk.

This version polls until the angles actually stop changing, and prints how long
each move took, so truncation would be visible rather than silent.

SAFETY: MOVES THE ARM over large angles (up to ~180 deg when scanning j1/j3).
Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_range.py --joint 3
    python3 tools/probe_range.py                  # all joints
    python3 tools/probe_range.py --joint 3 --step 10 --timeout 40
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# The one command proven to work on this unit.
BASE_CMD = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

# A move is considered finished once the angles have not changed by more than
# eps for `quiet` seconds. Slow joints just take longer; the timeout is a
# backstop, and every sample reports its elapsed time so truncation shows up.
EPS_DEG = 0.05
QUIET_S = 0.8


def raw(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def send(robot, cmd):
    try:
        robot.move_j(*[float(v) for v in cmd])
        return None
    except Exception as exc:  # noqa: BLE001
        return f'{type(exc).__name__}: {exc}'


def wait_settle(robot, timeout):
    """Poll until the readback stops changing. Returns (angles, elapsed)."""
    t0 = time.monotonic()
    last = raw(robot)
    stable_at = time.monotonic()
    while time.monotonic() - t0 < timeout:
        time.sleep(0.15)
        now = raw(robot)
        if max(abs(now[i] - last[i]) for i in range(6)) > EPS_DEG:
            stable_at = time.monotonic()
        elif time.monotonic() - stable_at > QUIET_S:
            return now, time.monotonic() - t0
        last = now
    return raw(robot), time.monotonic() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--joint', type=int, default=None,
                    help='joint number 1..6; omit to sweep all')
    ap.add_argument('--step', type=float, default=15.0,
                    help='grid step in degrees (default 15)')
    ap.add_argument('--span', type=float, default=180.0,
                    help='sweep from base-span to base+span (default 180)')
    ap.add_argument('--timeout', type=float, default=25.0,
                    help='max seconds to wait for one move to finish (default '
                         '25; raise it if the arm is slow)')
    ap.add_argument('--values', default=None,
                    help='explicit comma separated values, overrides the grid')
    args = ap.parse_args()

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except ImportError as exc:
        print(f'FATAL: cannot import dummy_cli_tool ({exc})')
        return 2

    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first '
              '(only one process can hold the fibre link).')
        return 1
    robot = dev.robot
    print('=== probe_range: which move_j values does the device accept? ===')
    print()
    print(f'timing: waits for the angles to settle (quiet {QUIET_S}s, '
          f'max {args.timeout}s per move); every sample reports its elapsed')
    print('        time, so a truncated wait would be visible.')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)

    err = send(robot, BASE_CMD)
    base_read, elapsed = wait_settle(robot, args.timeout)
    print(f'base command {BASE_CMD}')
    print(f'base readback {[round(v, 2) for v in base_read]}  '
          f'(settled in {elapsed:.1f}s)')
    if err:
        print(f'  base raised: {err}')
    print()

    # Note the mapping between the command space and the readback space: a
    # nonzero difference here is normal and is what the calibration encodes.
    print('  command space vs readback space (this is expected to differ):')
    for i in range(6):
        print(f'    joint_{i + 1}: sent {BASE_CMD[i]:>7.1f}   '
              f'read {base_read[i]:>8.2f}   diff '
              f'{base_read[i] - BASE_CMD[i]:>+8.2f}')
    print()

    joints = [args.joint] if args.joint else [3, 1, 2, 4, 5, 6]

    for jn in joints:
        idx = jn - 1
        if args.values:
            values = [float(v) for v in args.values.split(',')]
        else:
            values = []
            v = BASE_CMD[idx] - args.span
            while v <= BASE_CMD[idx] + args.span + 1e-9:
                values.append(round(v, 3))
                v += args.step

        print(f'--- joint_{jn}: sweeping its move_j value, others held at BASE')
        print(f'    {"value":>10} {"accepted":>9} {"settle s":>9} '
              f'{"readback after":>30}')
        accepted = []
        truncated = 0
        for v in values:
            # Return to BASE first so every sample starts from the same pose.
            send(robot, BASE_CMD)
            _, _ = wait_settle(robot, args.timeout)

            before = raw(robot)
            cmd = list(BASE_CMD)
            cmd[idx] = v
            err = send(robot, cmd)
            after, elapsed = wait_settle(robot, args.timeout)
            if elapsed >= args.timeout - 0.2:
                truncated += 1
            moved = max(abs(after[i] - before[i]) for i in range(6))
            ok = moved > 1.0 and err is None
            if ok:
                accepted.append(v)
            mark = 'YES' if ok else ('raise' if err else 'no')
            warn = '  <-- WAIT TRUNCATED' if elapsed >= args.timeout - 0.2 else ''
            print(f'    {v:>10.2f} {mark:>9} {elapsed:>9.1f} '
                  f'{[round(x, 2) for x in after]}{warn}')

        print()
        if accepted:
            lo, hi = min(accepted), max(accepted)
            print(f'    joint_{jn}: accepted values span [{lo:.1f}, {hi:.1f}]')
            gaps = [(accepted[i - 1], accepted[i])
                    for i in range(1, len(accepted))
                    if accepted[i] - accepted[i - 1] > args.step * 1.5]
            if gaps:
                print(f'    NOTE gaps inside that span: {gaps} -- the accepted')
                print('    set is not one contiguous interval.')
        else:
            print(f'    joint_{jn}: no grid value was accepted')
        if truncated:
            print(f'    WARNING: {truncated} sample(s) hit the {args.timeout}s')
            print('    timeout, so their readings may have been taken while the')
            print('    arm was still moving. Re-run with a larger --timeout.')
        print()

    print('=' * 62)
    print('The accepted values are what the driver must fold its targets into,')
    print('per joint. Anything outside is refused silently, which is exactly')
    print('the "sent ok, arm did not move" symptom.')
    print()
    print('joint_3 accepted 90 and 91.474 but not -270 or 450, so the device')
    print('does not treat whole-turn equivalents as interchangeable.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
