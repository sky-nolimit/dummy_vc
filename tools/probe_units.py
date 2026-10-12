#!/usr/bin/env python3
"""Work out what move_j actually accepts, since none of the enable paths help.

Established on the real device:

    robot.homing()                            moves the arm, perfectly
    move_j / set_position / set_position_with_time,
    with or without joint-level set_enable,
    with or without explicit velocity/current limits
                                              all return ok and do NOTHING

Same link, same motors, same process. So this is not an enable problem: the
motion command itself is being refused or ignored. The two live possibilities
are a unit mismatch (we send degrees, the device may want radians, making our
values out of range) and a latched fault.

This script separates those by sending values that are unambiguous in one unit
and absurd in the other, and by asking the device for its own state/limits
where it exposes them.

SAFETY: this moves the arm if anything works. Values are kept small (a few
degrees) or obviously-huge-but-clamped. Keep a hand on the power switch.

Run WITHOUT dummy_controller:

    pkill -f dummy_controller
    python3 tools/probe_units.py
"""
import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# A property the device may expose that names the accepted unit.
UNIT_HINTS = ['unit', 'units', 'angle_unit', 'use_radians', 'radians',
              'is_radians', 'degrees', 'is_degrees']


def angles(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=15.0)
    ap.add_argument('--settle', type=float, default=2.5)
    args = ap.parse_args()

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except ImportError as exc:
        print(f'FATAL: cannot import dummy_cli_tool ({exc})')
        return 2

    dev = ref_tool.find_any(path='usb', timeout=args.timeout)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first.')
        return 1
    robot = dev.robot
    print('=== probe_units: what does move_j actually accept? ===')
    print()

    print('1. unit / fault hints the device exposes')
    found = False
    for target, label in ((robot, 'robot'),
                          (getattr(robot, 'joint_1', None), 'joint_1')):
        if target is None:
            continue
        for n in UNIT_HINTS + ['error', 'errors', 'fault', 'state',
                               'is_enabled', 'enable']:
            if hasattr(target, n):
                found = True
                try:
                    print(f'    {label}.{n} = {getattr(target, n)!r}')
                except Exception as exc:  # noqa: BLE001
                    print(f'    {label}.{n} <error: {exc}>')
    if not found:
        print('    (none exposed)')
    print()

    print('2. what homing leaves behind, and what move_j does with it')
    print('    homing first, so the pose is known and repeatable')
    try:
        robot.homing()
        time.sleep(args.settle)
    except Exception as exc:  # noqa: BLE001
        print(f'    [FAIL] homing raised: {exc}')
        return 1
    base = angles(robot)
    print(f'    homed angles (deg) = {[round(v, 3) for v in base]}')
    if hasattr(robot, 'joint_1'):
        for n in ('position', 'angle'):
            if hasattr(robot.joint_1, n):
                try:
                    print(f'    joint_1.{n} = {getattr(robot.joint_1, n)!r}')
                except Exception as exc:  # noqa: BLE001
                    print(f'    joint_1.{n} <error: {exc}>')
    print()

    # Each trial: re-home, then send one specific value and measure.
    def trial(label, target):
        try:
            robot.homing()
            time.sleep(args.settle)
        except Exception as exc:  # noqa: BLE001
            print(f'  [FAIL] {label}: homing raised {exc}')
            return None
        before = angles(robot)
        try:
            robot.move_j(*target)
        except Exception as exc:  # noqa: BLE001
            print(f'  [FAIL] {label}: move_j raised {type(exc).__name__}: {exc}')
            return None
        time.sleep(args.settle)
        after = angles(robot)
        moves = [after[i] - before[i] for i in range(6)]
        biggest = max(abs(m) for m in moves)
        if biggest > 1.0:
            print(f'  [MOVE] {label}')
            print(f'         sent   {[round(v, 4) for v in target]}')
            print(f'         change {[round(m, 2) for m in moves]}')
        else:
            print(f'  [NOOP] {label}: max |delta| = {biggest:.3f} deg')
        return moves

    print('3. trials. Each is preceded by homing so they cannot contaminate')
    print('   each other. Small values first.')
    print()

    h = base
    d = 5.0                       # 5 units on joint 1
    rad = math.radians(5.0)       # the same 5 deg expressed in radians

    trial('joint_1 = +5.0 (as DEGREES, 5 deg)',
          [h[0] + d, h[1], h[2], h[3], h[4], h[5]])
    trial('joint_1 = +%.5f (as RADIANS, also 5 deg)' % rad,
          [h[0] + rad, h[1], h[2], h[3], h[4], h[5]])
    trial('joint_1 = -5.0 (negative, catches a sign/direction gate)',
          [h[0] - d, h[1], h[2], h[3], h[4], h[5]])
    trial('joint_2 = +5.0 (a different axis, in case j1 alone is stuck)',
          [h[0], h[1] + d, h[2], h[3], h[4], h[5]])
    trial('all six = homed + 5.0 (does a full-vector command behave?)',
          [h[i] + d for i in range(6)])

    print()
    print('=' * 62)
    print('Reading the result:')
    print('  - If the RADIANS line moves and the DEGREES line does not, then')
    print('    move_j takes radians and every value this project has sent has')
    print('    been ~57x too large, hence silently out of range.')
    print('  - If only a LATER axis moves, joint_1 specifically is stuck.')
    print('  - If NOTHING moves while homing works, the controller is most')
    print('    likely latched in a fault state: POWER CYCLE the arm (cut the')
    print('    mains, not just the process) and rerun.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
