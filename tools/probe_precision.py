#!/usr/bin/env python3
"""Test whether float precision in the command values breaks move_j.

Established: every LITERAL command moved; the one command built from a readback
did not. But a number is a number -- the device cannot know where it came from.
So the difference must be in the values themselves. Compare:

    probe_shape 3  [20.0,   0.0,  90.0,  0.0,    0.0,  0.0]      MOVED
    probe_shape 5  [19.99, 75.0, -90.0, -0.001, -0.0, -0.001]   NOOP

The literal is round numbers; the derived one carries the raw readback, i.e.
long mantissas and tiny values like -0.001 and -0.0. A fibre-style protocol
that serialises only a limited precision, or that chokes on a value very close
to zero, would behave exactly like this: the arm simply never moves.

This script sends the SAME physical command at several precisions, so if
precision is the culprit the pattern will be unmistakable.

SAFETY: MOVES THE ARM, up to 20 deg on joint_1. Keep a hand on the power
switch.

    pkill -f dummy_controller
    python3 tools/probe_precision.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

BASE = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
# The same destination as BASE but with joint_1 at 20 -- written cleanly.
TARGET = [20.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def raw(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def go(robot, cmd, wait=4.0):
    try:
        robot.move_j(*cmd)
        time.sleep(wait)
        return None
    except Exception as exc:  # noqa: BLE001
        return f'{type(exc).__name__}: {exc}'


def trial(robot, label, cmd):
    go(robot, BASE)
    before = raw(robot)
    err = go(robot, cmd)
    after = raw(robot)
    moved = max(abs(after[i] - before[i]) for i in range(6))
    tag = 'MOVED' if moved > 1.0 else 'NOOP '
    print(f'  [{tag}] {label}')
    print(f'         sent  {cmd}')
    print(f'         j1    {before[0]:.4f} -> {after[0]:.4f}  '
          f'(moved {moved:.3f})')
    if err:
        print(f'         raised {err}')
    return moved > 1.0


def main():
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
    print('=== probe_precision: do long mantissas stop move_j working? ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)
    go(robot, BASE)
    print(f'at BASE  {[round(v, 3) for v in raw(robot)]}')
    print()

    # The same loop, asking for the same joint_1 = 20 each time, but with the
    # OTHER joints written at decreasing precision, and with a long mantissa on
    # joint_1 itself.
    results = {}

    results['clean integers'] = trial(
        robot, 'clean: [20, 0, 90, 0, 0, 0]', TARGET)

    results['one decimal'] = trial(
        robot, 'one decimal: [20.0, 0.0, 90.0, 0.0, 0.0, 0.0]',
        [20.0, 0.0, 90.0, 0.0, 0.0, 0.0])

    results['long mantissa on j1'] = trial(
        robot, 'j1 with a long mantissa, others clean',
        [20.0000001234567, 0.0, 90.0, 0.0, 0.0, 0.0])

    results['tiny non-zero on j2'] = trial(
        robot, 'j2 = 0.001 instead of 0 (a near-zero nonzero value)',
        [20.0, 0.001, 90.0, 0.0, 0.0, 0.0])

    results['negative tiny on j4'] = trial(
        robot, 'j4 = -0.001 instead of 0',
        [20.0, 0.0, 90.0, -0.001, 0.0, 0.0])

    results['negative zero'] = trial(
        robot, 'j5 = -0.0 (negative zero)',
        [20.0, 0.0, 90.0, 0.0, -0.0, 0.0])

    # The exact shape the driver produced, for comparison.
    results['driver-shaped'] = trial(
        robot, 'driver-shaped: [19.99, 75, -90, -0.001, -0.0, -0.001]',
        [19.99, 75.0, -90.0, -0.001, -0.0, -0.001])

    print()
    print('=' * 62)
    print('Summary:')
    for k, v in results.items():
        print(f'  {k:<28} {"MOVED" if v else "NOOP"}')
    print()
    print('If "clean" moves and the ones with tiny/negative-zero/raw values do')
    print('not, the fix is to round and clean the move_j values before sending')
    print('(round to a fixed number of decimals, and map -0.0 and near-zero to')
    print('exactly 0.0).')
    print()
    print('If everything moves, precision is not the issue and the difference')
    print('must be in the target values themselves -- compare the printed')
    print('vectors to see which joint disagreed.')


if __name__ == '__main__':
    sys.exit(main())
