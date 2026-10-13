#!/usr/bin/env python3
"""Isolate the exact condition that makes move_j a no-op, one variable at a time.

What the probes have established, all with literal commands from a known base:

    [20, 0, 90, 0, 0, 0]                   MOVED   (j1, j2, j3 all travel)
    [20, 0, 90, 0, 0, 0] with j2=0.001     MOVED
    [20, 0, 90, 0, 0, 0] with j4=-0.001    MOVED
    [20, 0, 90, 0, 0, 0] with j5=-0.0      MOVED
    [20.0000001234567, 0, 90, 0, 0, 0]     MOVED
    [19.99, 75, -90, -0.001, -0.0, -0.001] NOOP

Precision, negative zero and long mantissas are therefore ruled out. The last
line differs in that joints 2 and 3 are told to stay where they already are,
so only joint 1 has to travel.

This script varies exactly that, with everything else held fixed, and always
starts from the same literal base pose. Each trial prints how many joints have
somewhere to go, and how many actually moved, so the two can be compared
directly.

SAFETY: MOVES THE ARM, up to about 75 deg on joints 2/3 (the same motion the
earlier probes already performed). Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_condition.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Where the arm sits after this literal command, per its own readback:
#   joint_1 ~ 0, joint_2 ~ 75, joint_3 ~ -90, rest ~ 0
BASE = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def raw(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def go(robot, cmd, wait=4.0):
    try:
        robot.move_j(*[float(v) for v in cmd])
        time.sleep(wait)
        return None
    except Exception as exc:  # noqa: BLE001
        return f'{type(exc).__name__}: {exc}'


def trial(robot, label, cmd):
    go(robot, BASE)
    before = raw(robot)
    need = [abs(float(cmd[i]) - before[i]) for i in range(6)]
    n_need = sum(1 for d in need if d > 1.0)
    err = go(robot, cmd)
    after = raw(robot)
    moved = [after[i] - before[i] for i in range(6)]
    n_moved = sum(1 for d in moved if abs(d) > 1.0)
    tag = 'MOVED' if n_moved else 'NOOP '
    print(f'  [{tag}] {label}')
    print(f'         sent       {[round(float(v), 3) for v in cmd]}')
    print(f'         before     {[round(v, 2) for v in before]}')
    print(f'         after      {[round(v, 2) for v in after]}')
    print(f'         travel required: {n_need} joint(s) | moved: {n_moved}')
    if err:
        print(f'         raised {err}')
    print()
    return n_need, n_moved


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
    print('=== probe_condition: one variable at a time ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)
    go(robot, BASE)
    print(f'base pose readback: {[round(v, 2) for v in raw(robot)]}')
    print('(the readback is in the device\'s own space, so compare the'
          ' "travel required" count, not the numbers)')
    print()

    # These are the raw readback values at BASE. A target that reproduces them
    # asks those joints to stay put.
    stay = raw(robot)
    stay = [round(v, 3) for v in stay]

    # Joint 1 always travels to 20. The other joints either travel with it or
    # are told to stay.
    tests = [
        ('all four others told to stay (driver shape)',
         [20.0, stay[1], stay[2], stay[3], stay[4], stay[5]]),
        ('j2 also travels (to 0), j3 stays',
         [20.0, 0.0, stay[2], stay[3], stay[4], stay[5]]),
        ('j3 also travels (to 90), j2 stays',
         [20.0, stay[1], 90.0, stay[3], stay[4], stay[5]]),
        ('j2 and j3 travel, j4-j6 stay',
         [20.0, 0.0, 90.0, stay[3], stay[4], stay[5]]),
        ('j2, j3, j4 travel',
         [20.0, 0.0, 90.0, 0.0, stay[4], stay[5]]),
        ('j2 travels only a little (to 70)',
         [20.0, 70.0, stay[2], stay[3], stay[4], stay[5]]),
        ('j3 travels only a little (to -85)',
         [20.0, stay[1], -85.0, stay[3], stay[4], stay[5]]),
    ]

    rows = []
    for label, cmd in tests:
        rows.append((label,) + trial(robot, label, cmd))

    print('=' * 62)
    print(f'  {"test":<44} {"travel":>7} {"moved":>6}')
    for label, n_need, n_moved in rows:
        print(f'  {label:<44} {n_need:>7} {n_moved:>6}')
    print()
    print('Read the "travel" column: whichever value of it stops producing')
    print('movement is the threshold the device applies. If even "travel 1"')
    print('moves here, then nothing about the count is at fault and the')
    print('driver-shaped vector differs some other way -- the printed after')
    print('values in that row will show it.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
