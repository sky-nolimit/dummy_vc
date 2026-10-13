#!/usr/bin/env python3
"""Find the value range move_j accepts, per joint.

The user's observation, confirmed by the sweep:

    j3 =   90      accepted   (the working literal)
    j3 =   91.474  accepted   (the calibrated value)
    j3 =  -270      refused    <- mathematically the same angle
    j3 =  -268.526  refused    <- what the driver currently sends
    j3 =  450       refused
    j3 =    0       refused
    j3 =  -90       refused
    j3 =  270       refused

So the device does NOT fold these into one another. It has a hard accepted
range, and values outside it are refused SILENTLY -- the whole command is
dropped, which is why the arm does not move at all.

That has a direct consequence for the driver: normalising into [0, 360), or to
the whole-turn equivalent nearest the readback, are both wrong. The target must
be folded into whatever range the device accepts, per joint.

This script maps that range. For each joint it holds the others at the working
literal and sweeps the joint of interest across a grid, reporting whether the
command was accepted (detected by the readback changing for any joint, since an
accepted command moves the arm).

Note the axes are in different spaces: the command value and the readback value
differ, so acceptance is judged by "did anything move", not by comparing the
commanded value to the readback.

SAFETY: MOVES THE ARM. The sweep can send the arm to unusual poses; the grid is
kept to values near the working pose and the motion per step is bounded by
commanding the same base first. Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_range.py
    python3 tools/probe_range.py --joint 3 --step 10
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
BASE = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
# At BASE the arm is at this command pose; a joint's own BASE value is what the
# sweep varies.
BASE_CMD = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def raw(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def send(robot, cmd):
    try:
        robot.move_j(*[float(v) for v in cmd])
        return None
    except Exception as exc:  # noqa: BLE001
        return f'{type(exc).__name__}: {exc}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--joint', type=int, default=None,
                    help='joint number 1..6; omit to sweep all')
    ap.add_argument('--step', type=float, default=15.0,
                    help='grid step in degrees (default 15)')
    ap.add_argument('--span', type=float, default=180.0,
                    help='sweep from -span to +span around each joints base')
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
    print('=== probe_range: which move_j values does the device accept? ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)

    err = send(robot, BASE)
    time.sleep(args.settle)
    print(f'base   sent {BASE}')
    print(f'       read {[round(v, 2) for v in raw(robot)]}'
          f'{"  (raised " + err + ")" if err else ""}')
    print()

    joints = [args.joint] if args.joint else [3, 1, 2, 4, 5, 6]

    for jn in joints:
        idx = jn - 1
        base_val = BASE_CMD[idx]
        print(f'--- joint_{jn}: sweeping its move_j value, others held at BASE')
        print(f'    {"value":>10}  {"accepted?":>10}  {"readback after":>28}')
        accepted = []
        vals = []
        v = base_val - args.span
        while v <= base_val + args.span + 1e-9:
            # Return to BASE first so each sample starts from the same pose.
            # If BASE itself is refused, the arm simply stays put, which does
            # not invalidate the sample.
            send(robot, BASE)
            time.sleep(0.8)
            before = raw(robot)
            cmd = list(BASE_CMD)
            cmd[idx] = v
            err = send(robot, cmd)
            time.sleep(args.settle)
            after = raw(robot)
            moved = max(abs(after[i] - before[i]) for i in range(6))
            ok = moved > 1.0 and err is None
            if ok:
                accepted.append(v)
            vals.append((v, ok))
            mark = 'YES' if ok else ('raise' if err else 'no')
            print(f'    {v:>10.2f}  {mark:>10}  {[round(x, 2) for x in after]}')
            v += args.step

        print()
        if accepted:
            lo, hi = min(accepted), max(accepted)
            print(f'    joint_{jn}: accepted values span [{lo:.1f}, {hi:.1f}]'
                  f' over the grid sampled')
            gaps = []
            for i in range(1, len(accepted)):
                if accepted[i] - accepted[i - 1] > args.step * 1.5:
                    gaps.append((accepted[i - 1], accepted[i]))
            if gaps:
                print(f'    NOTE gaps inside that span: {gaps} -- the accepted')
                print('    set is not a single contiguous interval.')
        else:
            print(f'    joint_{jn}: NO grid value was accepted')
        print()

    print('=' * 62)
    print('The accepted values are what the driver must fold its targets into,')
    print('per joint. Anything outside is refused silently, which is exactly')
    print('the "sent ok, arm did not move" symptom.')
    print()
    print('Note joint_3 accepted 90 and 91.474 but not -270 or 450, so the')
    print('device does not treat whole-turn equivalents as interchangeable.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
