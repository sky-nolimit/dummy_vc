#!/usr/bin/env python3
"""List what the device actually exposes, and try the obvious motion paths.

Why: robot.move_j(...) returns without raising yet nothing moves. That means
either a call is a silent no-op, or some enable/mode step is missing. Guessing
further is wasteful -- this dumps the real interface and exercises each
plausible command in turn, reporting what each one does to the angle readback.

It is strictly diagnostic and it DOES move the arm if a command works, so keep
a hand on the power switch. Default is the smallest useful move.

Run WITHOUT dummy_controller (one process per fibre link):

    pkill -f dummy_controller
    python3 tools/probe_interface.py                 # list only, no motion
    python3 tools/probe_interface.py --try-all       # also attempt moves
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Attributes that look like they control whether motion is accepted.
INTERESTING = [
    'set_enable', 'enable', 'is_enabled', 'enabled',
    'move_j', 'move_l', 'move_c', 'move_p',
    'homing', 'homing_', 'resting',
    'set_position', 'set_position_with_time', 'set_velocity', 'set_current',
    'erasing', 'erase', 'calibration', 'calibrate',
    'state', 'axis_state', 'error', 'errors', 'estop', 'clear_errors',
    'position', 'angle', 'vel', 'velocity',
    'set_current_limit', 'set_velocity_limit',
]


def describe(name, obj):
    kind = type(obj).__name__
    if callable(obj):
        try:
            import inspect
            sig = str(inspect.signature(obj))
        except (TypeError, ValueError):
            sig = '(?)'
        return f'{name}{sig}'
    return f'{name} = {obj!r}  [{kind}]'


def dump(obj, label, names):
    print(f'--- {label}')
    for n in names:
        if hasattr(obj, n):
            try:
                print(f'    {describe(n, getattr(obj, n))}')
            except Exception as exc:  # noqa: BLE001
                print(f'    {n}  <error reading: {exc}>')
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--try-all', action='store_true',
                    help='also attempt each motion command (MOVES THE ARM)')
    ap.add_argument('--delta', type=float, default=5.0)
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
    print('=== probe_interface ===')
    print()

    # ---- 1. what does the robot object expose ---------------------------
    print('1. attributes on the robot object that could gate motion')
    dump(robot, 'robot', INTERESTING)

    print('2. attributes on joint_1 and on the hand')
    dump(getattr(robot, 'joint_1'), 'robot.joint_1', INTERESTING)
    dump(getattr(robot, 'hand', None), 'robot.hand', INTERESTING)

    # ---- 3. current state ------------------------------------------------
    print('3. current state')
    angles = [float(getattr(robot, n).angle) for n in JR]
    print(f'    raw angles (deg) = {[round(v, 3) for v in angles]}')
    for n in ('enable', 'is_enabled', 'enabled', 'state', 'error', 'errors',
              'estop'):
        if hasattr(robot, n):
            try:
                print(f'    robot.{n} = {getattr(robot, n)!r}')
            except Exception as exc:  # noqa: BLE001
                print(f'    robot.{n} <error: {exc}>')
    print()

    if not args.try_all:
        print('=' * 62)
        print('Listed only (no motion attempted).')
        print('Re-run with --try-all to exercise each motion command.')
        print('Keep a hand on the power switch if you do.')
        return 0

    # ---- 4. try each motion path ----------------------------------------
    print('4. attempting motion paths (hand on the power switch)')
    print()

    def attempt(label, fn):
        before = [float(getattr(robot, n).angle) for n in JR]
        try:
            fn()
            err = None
        except Exception as exc:  # noqa: BLE001
            err = f'{type(exc).__name__}: {exc}'
        time.sleep(args.settle)
        try:
            after = [float(getattr(robot, n).angle) for n in JR]
        except Exception as exc:  # noqa: BLE001
            print(f'  [??  ] {label}: readback failed: {exc}')
            return None
        moves = [after[i] - before[i] for i in range(6)]
        biggest = max(abs(m) for m in moves)
        if err:
            print(f'  [FAIL] {label}: raised {err}')
        elif biggest < 0.3:
            print(f'  [NOOP] {label}: returned ok, nothing moved '
                  f'(max |delta| = {biggest:.3f} deg)')
        else:
            print(f'  [MOVE] {label}: max |delta| = {biggest:.3f} deg '
                  f'{[round(m, 2) for m in moves]}')
        return moves

    # Keep every other joint where it is and nudge only joint_1.
    base = [float(getattr(robot, n).angle) for n in JR]
    target = list(base)
    target[0] = base[0] + args.delta
    print(f'  target: joint_1 {base[0]:.3f} -> {target[0]:.3f} '
          f'(others unchanged)')
    print()

    attempt('set_enable(True) then move_j', lambda: (
        robot.set_enable(True),
        robot.move_j(*target),
    ))

    attempt('move_j alone (already enabled)', lambda: robot.move_j(*target))

    if hasattr(robot.joint_1, 'set_position'):
        attempt('joint_1.set_position',
                lambda: robot.joint_1.set_position(target[0]))

    if hasattr(robot.joint_1, 'set_position_with_time'):
        attempt('joint_1.set_position_with_time(pos, 1.0)',
                lambda: robot.joint_1.set_position_with_time(target[0], 1.0))

    if hasattr(robot, 'homing'):
        attempt('robot.homing()', lambda: robot.homing())

    print()
    print('=' * 62)
    print('Read the [MOVE] lines: whichever call actually moves the arm is the')
    print('one the driver should be using. If every line is [NOOP] or [FAIL],')
    print('the device is accepting commands but its power stage is not acting,')
    print('which is a hardware or firmware issue, not a code one.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
