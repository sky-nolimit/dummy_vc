#!/usr/bin/env python3
"""Record move_j -> raw pairs, so the calibration is read off the device.

Everything about the joints is derivable from one thing: the mapping between
the value put into move_j and the raw angle that comes back. The project never
measured it properly -- it was reconstructed from a pose assumed to be known,
and that assumption is where the 75 deg and 90 deg discrepancies on joint_2 and
joint_3 came from.

This script produces exactly that table, for every joint, with the arm given
time to settle. Each row is a fact: "this is what I asked for, this is what
came back". Nothing is inferred.

Two commands are also included because they are the reference poses the rest of
the configuration is written around:

    [0, 0, 90, 0, 0, 0]      home_joints_deg
    [0, -75, 180, 0, 0, 0]   reset_joints_deg

Their readbacks pin the offsets for joint_2 and joint_3 directly, without any
derivation at all.

SAFETY: MOVES THE ARM. Sweeps are inside the measured accepted ranges, one
joint at a time, always returning to the base pose first. Keep a hand on the
power switch.

    pkill -f dummy_controller
    python3 tools/probe_pairs.py                  # reference poses + sweeps
    python3 tools/probe_pairs.py --refs-only      # just the two named poses
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

BASE = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

# The two poses the configuration is written around.
REFERENCE_POSES = [
    ('home_joints_deg', [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]),
    ('reset_joints_deg', [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]),
]

# per-joint values to sample, all inside the ranges probe_range measured
GRIDS = {
    1: [-90.0, -45.0, 45.0, 90.0],
    2: [-60.0, -30.0, 30.0, 60.0, 90.0],
    3: [45.0, 75.0, 120.0, 150.0, 180.0],
    4: [-90.0, 90.0],
    5: [-60.0, -30.0, 30.0, 60.0],
    6: [-90.0, 90.0],
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--refs-only', action='store_true')
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--out', default='movej_raw_pairs.json')
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
    print('=== probe_pairs: move_j -> raw, measured ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)

    records = []

    def run(cmd, label):
        err = send(robot, cmd)
        got, elapsed = wait_settle(robot, args.timeout)
        print(f'  {label:<22} cmd {[round(v, 1) for v in cmd]}')
        print(f'  {"":<22} raw {[round(v, 3) for v in got]}   '
              f'({elapsed:.1f}s){"  RAISED " + err if err else ""}')
        records.append({'label': label, 'cmd': [float(v) for v in cmd],
                        'raw': [float(v) for v in got]})
        print()

    print('--- reference poses (these pin the offsets directly)')
    for label, cmd in REFERENCE_POSES:
        run(cmd, label)

    if not args.refs_only:
        print('--- per-joint sweeps, everything else held at the base pose')
        for jn in range(1, 7):
            idx = jn - 1
            for v in GRIDS.get(jn, []):
                send(robot, BASE)
                wait_settle(robot, args.timeout)
                cmd = list(BASE)
                cmd[idx] = v
                run(cmd, f'joint_{jn} = {v:.0f}')

    with open(args.out, 'w', encoding='utf-8') as fh:
        json.dump(records, fh, indent=2)
    print(f'saved {len(records)} records to {args.out}')
    print()

    print('=' * 72)
    print('HOW TO READ THIS')
    print('=' * 72)
    print('  Each row is a fact: I commanded move_j X, the device read back Y.')
    print()
    print('  The reference poses are the important ones. The configuration is')
    print('  written around them, so:')
    print()
    for rec in records:
        if rec['label'].startswith('home') or rec['label'].startswith('reset'):
            print(f'    {rec["label"]}')
            for i in range(6):
                d = rec['raw'][i] - rec['cmd'][i]
                print(f'      joint_{i + 1}: move_j {rec["cmd"][i]:>7.2f}  '
                      f'raw {rec["raw"][i]:>9.3f}  raw - move_j '
                      f'{d:>+9.3f}')
            print()
    print('  `raw - move_j` per joint, at a pose the config names, is the')
    print('  offset the calibration has to reproduce. If the config disagrees')
    print('  with these numbers, the config is what is wrong.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
