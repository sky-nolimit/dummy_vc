#!/usr/bin/env python3
"""Map which j3 values move_j actually accepts.

The user spotted what I had been overlooking: in the failing vector the j3
value was -90 while every working vector used +90. Those differ by a whole turn
only in some interpretations, and the device clearly treats some spellings as
acceptable and others as not.

Where the odd value came from is now clear, and it was my bug: probe_shape step
5 built its target by copying the device's RAW readback

    before = [joint.angle for ...]      -> [-0.01, 75, -90, ...]
    cmd[0] = before[0] + 20
    move_j(*cmd)                        -> [19.99, 75, -90, -0.001, -0.0, -0.001]

but the raw readback and the move_j command space are NOT the same (the user
established that move_j j3 = 90 puts the raw readback at about -90). So I fed
readback values back in as commands, which is a type error. That is why the
vector looked wrong: j2 = 75 and j3 = -90 are literally "stay where you are" in
the raw space, and they do not mean that in the command space.

What still needs pinning down, and is the real question for the driver, is the
range of j3 values move_j accepts, because the driver currently computes
-268.53 and the working literals use +90.

This script sweeps j3 while holding everything else at a known-good literal,
and reports which values produce movement. It also sweeps j1 to confirm the
harness itself is working.

SAFETY: MOVES THE ARM. j1 sweeps within +/-20 deg; j3 is swept across
equivalents of the same physical pose so the arm should not travel far, but
keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_j3range.py
"""
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


def go(robot, cmd, wait=4.0):
    try:
        robot.move_j(*[float(v) for v in cmd])
        time.sleep(wait)
        return None
    except Exception as exc:  # noqa: BLE001
        return f'{type(exc).__name__}: {exc}'


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
    print('=== probe_j3range: which j3 spellings does move_j accept? ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)
    time.sleep(1.0)
    go(robot, BASE)
    base = raw(robot)
    print(f'base command {BASE}')
    print(f'base readback {[round(v, 3) for v in base]}')
    print()
    print('Note: the readback for j3 is NEGATIVE while the command value was')
    print('+90, which is exactly why feeding readback values back in as')
    print('commands is wrong.')
    print()

    # --- sweep j1, to prove the harness works at all ---------------------
    print('--- control sweep: j1 only (should move)')
    for j1 in (20.0, 0.0, 20.0):
        go(robot, BASE)
        before = raw(robot)
        go(robot, [j1, 0.0, 90.0, 0.0, 0.0, 0.0])
        after = raw(robot)
        delta = after[0] - before[0]
        print(f'    j1 -> {j1:>6.2f}:  readback {before[0]:>8.3f} -> '
              f'{after[0]:>8.3f}  (moved {delta:+.3f})  '
              f'{"OK" if abs(delta) > 1 else "NOOP"}')
    print()

    # --- sweep j3 through whole-turn equivalents of the same pose --------
    print('--- j3 sweep, everything else held at the working literal')
    print('    (all of these mean the same physical pose for j3)')
    print(f'    {"j3 sent":>12}  {"j3 readback":>12}  {"j1 moved?":>10}  note')
    variants = [
        (90.0, 'the working literal'),
        (91.474, 'the calibrated 7-pose value'),
        (-270.0, '90 - 360'),
        (-268.526, '91.474 - 360'),
        (450.0, '90 + 360'),
        (0.0, 'the retracted-ish value'),
        (-90.0, 'what my buggy probe copied from the readback'),
        (270.0, '90 + 180, i.e. the half-turn relative'),
    ]
    for j3, note in variants:
        go(robot, BASE)
        before = raw(robot)
        go(robot, [20.0, 0.0, j3, 0.0, 0.0, 0.0])
        after = raw(robot)
        d1 = after[0] - before[0]
        d3 = after[2] - before[2]
        verdict = 'OK' if abs(d1) > 1 else 'NOOP'
        print(f'    {j3:>12.3f}  {after[2]:>12.3f}  {verdict:>10}  {note}'
              f'  (j3 moved {d3:+.2f})')
    print()

    print('=' * 62)
    print('If only some of the j3 spellings move the arm, the accepted range')
    print('is what the driver must normalise into -- and -268.53, which the')
    print('driver currently sends, is outside it. The fix is then to wrap j3')
    print('into the range this sweep shows as accepted, rather than into')
    print('[0, 360) or to the nearest equivalent of the raw readback.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
