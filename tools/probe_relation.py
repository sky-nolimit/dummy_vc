#!/usr/bin/env python3
"""Measure raw <-> move_j for every joint, in ONE consistent way.

Why this is needed: joint 3 currently satisfies the readback chain but not the
target chain, using the same two parameters:

    readback : urdf = sign * (angle - joint_offset)      -> correct at the 7 pose
    target   : move_j = urdf/sign + joint_offset + move_j_offset
                                                          -> lands 91.474 deg off

Both cannot be right with one set of numbers, so one of the two formulas is
mis-modelling the device. The only way out is to stop deriving and measure the
thing both formulas are guessing at: the direct relation between the value we
put into move_j and the raw angle that comes back.

    raw_angle = a * command_move_j + b

From that single relation, per joint:
    * `a` is the gain, which pins the sign (a > 0 means same direction);
    * `b` pins the offset;
    * and both chains can then be reconstructed so that they agree.

Method: command ONE joint to a series of values with everything else held at a
base pose, wait for the arm to actually settle (polling, not sleeping), and
record (commanded, readback) pairs. Nothing is derived from the readback except
the pair itself, which is the whole point.

SAFETY: MOVES THE ARM, one joint at a time, within the base pose's neighbours.
Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_relation.py
    python3 tools/probe_relation.py --joint 3
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# The pose the device reaches on homing, written as the command that gets there.
BASE_CMD = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

# sampled command values per joint, chosen inside the ranges probe_range found
GRIDS = {
    1: [-90.0, -45.0, 0.0, 45.0, 90.0],
    2: [-60.0, -30.0, 0.0, 30.0, 60.0],
    3: [45.0, 60.0, 90.0, 120.0, 150.0, 180.0],
    4: [-90.0, -45.0, 45.0, 90.0],
    5: [-60.0, -30.0, 30.0, 60.0],
    6: [-90.0, -45.0, 45.0, 90.0],
}

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
    t0 = time.monotonic()
    last = raw(robot)
    stable = time.monotonic()
    while time.monotonic() - t0 < timeout:
        time.sleep(0.15)
        now = raw(robot)
        if max(abs(now[i] - last[i]) for i in range(6)) > EPS_DEG:
            stable = time.monotonic()
        elif time.monotonic() - stable > QUIET_S:
            return now, time.monotonic() - t0
        last = now
    return raw(robot), time.monotonic() - t0


def fit(pairs):
    """Least squares raw = a * cmd + b, and the residuals."""
    n = len(pairs)
    sx = sum(c for c, _ in pairs)
    sy = sum(r for _, r in pairs)
    sxx = sum(c * c for c, _ in pairs)
    sxy = sum(c * r for c, r in pairs)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None, None, []
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    resid = [r - (a * c + b) for c, r in pairs]
    return a, b, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--joint', type=int, default=None)
    ap.add_argument('--timeout', type=float, default=30.0)
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
    print('=== probe_relation: raw angle vs commanded move_j, measured ===')
    print()
    print('waits for the arm to settle (polling), never a fixed sleep;')
    print('every sample is a (commanded, readback) pair with nothing derived')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)
    send(robot, BASE_CMD)
    wait_settle(robot, args.timeout)
    print(f'base command {BASE_CMD}')
    print(f'base readback {[round(v, 2) for v in raw(robot)]}')
    print()

    joints = [args.joint] if args.joint else [1, 2, 3, 4, 5, 6]
    summary = {}

    for jn in joints:
        idx = jn - 1
        print(f'--- joint_{jn}')
        print(f'    {"commanded":>10} {"readback":>12} {"settle s":>9}')
        pairs = []
        for v in GRIDS.get(jn, []):
            send(robot, BASE_CMD)
            wait_settle(robot, args.timeout)
            cmd = list(BASE_CMD)
            cmd[idx] = v
            err = send(robot, cmd)
            got, elapsed = wait_settle(robot, args.timeout)
            print(f'    {v:>10.2f} {got[idx]:>12.3f} {elapsed:>9.1f}'
                  f'{"   RAISED " + err if err else ""}')
            pairs.append((v, got[idx]))

        a, b, resid = fit(pairs)
        print()
        if a is None:
            print('    not enough independent samples to fit')
        else:
            print(f'    raw = {a:+.5f} * command {b:+.3f}')
            print(f'    residuals = {[round(r, 3) for r in resid]}')
            sn = 1 if a > 0 else -1
            print(f'    gain {a:+.5f} -> sign {"+1" if sn > 0 else "-1"}')
            print(f'    offset at command 0: raw = {b:+.3f}')
            summary[jn] = (a, b, sn)

            # Invert for the other direction.
            if abs(a) > 1e-9:
                print(f'    inverse: command = (raw - ({b:+.3f})) / {a:+.5f}')
        print()

    print('=' * 72)
    print('WHAT THIS GIVES, PER JOINT')
    print('=' * 72)
    for jn in sorted(summary):
        a, b, sn = summary[jn]
        print(f'  joint_{jn}: raw = {a:+.4f}*cmd {b:+.3f}   (sign {sn:+d})')
    print()
    print('  The two chains the driver uses must be built from THIS relation:')
    print('      raw  -> URDF   :  urdf = sign * (raw - raw_at_urdf_zero)')
    print('      URDF -> move_j :  move_j = raw_at_urdf_zero + urdf/sign')
    print('  where raw_at_urdf_zero is the raw angle of the calibration pose.')
    print()
    print('  Deriving them from the measured line is what makes them agree.')
    print('  Deriving them from a pose we only assume we know is what produced')
    print('  the 91.474 deg disagreement on joint_3.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
