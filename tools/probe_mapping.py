#!/usr/bin/env python3
"""Measure, directly, the relation between a move_j command and the readback.

The user asked the question that exposes the core confusion:

    "we send move_j and the arm moves by that much, so why derive anything?"

and they are right to push. Two different formulas are currently used to
describe the same device:

    raw -> URDF                 urdf_deg = sign * (raw - joint_offset_deg)
    raw -> move_j (current)     move_j   = raw - move_j_offset_deg

Those disagree, and the disagreement is joint dependent. Worse, a single real
command already shows three different relations:

    move_j sent [20, 0, 90, 0, 0, 0]
    raw read   [20, 75, -90, 0, 0, 0]
        joint_1: command 20  -> readback  20   (1:1)
        joint_2: command  0  -> readback  75   (offset 75)
        joint_3: command 90  -> readback -90   (offset -180)

So rather than assume anything, this measures each joint's mapping directly:
command one joint through a set of values, record the readback each time, and
report the relation. From that, both constants (gain and offset) are read off
the data instead of being fitted or guessed.

It also reveals whether the mapping is even linear, and whether it wraps.

SAFETY: MOVES THE ARM. Only one joint is moved per sample, values stay near the
proven-good pose, and it returns to base between samples. Keep a hand on the
power switch.

    pkill -f dummy_controller
    python3 tools/probe_mapping.py
    python3 tools/probe_mapping.py --joint 3 --values 60,75,90,105,120
"""
import argparse
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


def send(robot, cmd):
    try:
        robot.move_j(*[float(v) for v in cmd])
        return None
    except Exception as exc:  # noqa: BLE001
        return f'{type(exc).__name__}: {exc}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--joint', type=int, default=None,
                    help='joint to map, 1..6; omit for all')
    ap.add_argument('--values', default=None,
                    help='comma separated command values to try')
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
    print('=== probe_mapping: command value vs readback, measured ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)
    send(robot, BASE)
    time.sleep(args.settle)
    print(f'base command {BASE}')
    print(f'base readback {[round(v, 2) for v in raw(robot)]}')
    print()
    print('For each joint we vary only that joint\'s command value, holding the')
    print('others at the base, and record what the readback does. That gives')
    print('the gain and offset for that joint straight from the hardware.')
    print()

    if args.values:
        grid = [float(v) for v in args.values.split(',')]
    else:
        grid = None

    joints = [args.joint] if args.joint else [1, 2, 3, 4, 5, 6]
    findings = {}

    for jn in joints:
        idx = jn - 1
        if grid is not None:
            values = grid
        elif jn == 3:
            values = [60.0, 75.0, 90.0, 105.0, 120.0]
        elif jn == 2:
            values = [-15.0, 0.0, 15.0, 30.0]
        else:
            values = [-20.0, -10.0, 0.0, 10.0, 20.0]

        print(f'--- joint_{jn} (command value -> that joint\'s readback)')
        print(f'    {"sent":>8} {"readback":>10} {"delta from base":>18}')
        base_read = None
        samples = []
        for v in values:
            send(robot, BASE)
            time.sleep(0.8)
            cmd = list(BASE)
            cmd[idx] = v
            err = send(robot, cmd)
            time.sleep(args.settle)
            got = raw(robot)[idx]
            if base_read is None:
                base_read = got
            samples.append((v, got))
            print(f'    {v:>8.2f} {got:>10.3f} {got - base_read:>18.3f}'
                  f'{"   RAISED " + err if err else ""}')
        print()

        # Read the gain and offset off the data where possible.
        if len(samples) >= 2:
            (v0, r0), (v1, r1) = samples[0], samples[-1]
            dv, dr = v1 - v0, r1 - r0
            if abs(dv) > 1e-9:
                gain = dr / dv
                offset = r0 - gain * v0
                print(f'    over the sampled range: gain = {gain:+.4f}, '
                      f'offset = {offset:+.3f}')
                print(f'    i.e. readback ~= {gain:+.4f} * command '
                      f'{offset:+.3f}')
                findings[jn] = (gain, offset)
                if abs(abs(gain) - 1.0) > 0.15:
                    print('    NOTE gain is far from +/-1: command and readback')
                    print('    are not the same quantity for this joint.')
        print()

    print('=' * 62)
    print('Summary of what the hardware actually does:')
    for jn in sorted(findings):
        gain, offset = findings[jn]
        print(f'  joint_{jn}: readback = {gain:+.4f} * command {offset:+.3f}')
    print()
    print('If gain is -1 for some joints and +1 for others, that is the sign.')
    print('The offset here is what the calibration should use -- read from the')
    print('device, not fitted from a pose we assume we know.')
    print()
    print('For joint_3 specifically: earlier data suggests command 90 gives a')
    print('readback near -90, which would make the offset about -180 and the')
    print('sign +1. If this run confirms that, the value currently in the')
    print('config (-91.474) is wrong for that joint.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
