#!/usr/bin/env python3
"""Test whether the driver's [0,360) wrapping is what breaks its commands.

Established so far, all with no homing() in the loop:

    move_j(0, 0, 90, 0, 0, 0)      from [0.57, 0.15, ...]   MOVED 89 deg
    move_j(20, 0, 90, 0, 0, 0)     from [0, 75, ...]        MOVED 20 deg
    move_j(20, 0, 90, 0, 0, 0)     from [20, 75, ...]       NOOP (already there)
    move_j(current + 8 on joint_1) from the same pose       NOOP

So the device ignores a command whose target IS the current pose, and accepts
commands whose target differs. move_j works.

Which raises the question the driver actually fails on. Its log shows:

    waypoint 1/1 deg=[339.78, 0.43, 91.47, 0.91, 0.04, 0.04]

339.78, while the device sits at ~0. The driver normalises move_j into
[0, 360), so a target of -20.22 deg is sent as 339.78 deg. If the device
interprets that literally, it is being asked to travel almost a full turn --
very likely past the joint limit -- and a rejected move looks exactly like a
silent no-op. The probes above never wrapped, which is why they worked.

This script sends the SAME physical target three ways and compares:

    W  wrapped into [0, 360)          <- what the driver does
    N  nearest equivalent to current  <- the standard fix
    P  plain, unwrapped

If W is ignored while N and P move, the fix is to stop wrapping into [0,360)
and instead pick the whole-turn equivalent closest to the current position.

SAFETY: MOVES THE ARM. Targets are the same pose, i.e. joint_1 by -20 deg.
Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_wrap.py
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
HOME = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def angles(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def wrap360(v):
    while v < 0.0:
        v += 360.0
    while v >= 360.0:
        v -= 360.0
    return v


def nearest(v, ref):
    """The whole-turn equivalent of v that is closest to ref."""
    return v + 360.0 * round((ref - v) / 360.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--settle', type=float, default=3.0)
    ap.add_argument('--delta', type=float, default=-20.0,
                    help='joint_1 target offset from home (default -20)')
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
    print('=== probe_wrap: is [0,360) wrapping what breaks the driver? ===')
    print()

    robot.set_enable(1)
    robot.set_rgb_mode(4)

    def go_home():
        robot.move_j(*HOME)
        time.sleep(args.settle)

    go_home()
    print(f'enabled, at home. angles = {[round(v, 2) for v in angles(robot)]}')
    print()

    # The physical target: joint_1 at args.delta degrees.
    plain = list(HOME)
    plain[0] = args.delta
    print(f'physical target  = {plain}')
    print()

    def trial(label, target):
        go_home()
        before = angles(robot)
        try:
            robot.move_j(*target)
        except Exception as exc:  # noqa: BLE001
            print(f'  [FAIL] {label}: raised {type(exc).__name__}: {exc}')
            return None
        time.sleep(args.settle)
        after = angles(robot)
        moves = [after[i] - before[i] for i in range(6)]
        biggest = max(abs(m) for m in moves)
        tag = 'MOVE' if biggest > 1.0 else 'NOOP'
        print(f'  [{tag}] {label}')
        print(f'         sent   {[round(v, 2) for v in target]}')
        print(f'         change {[round(m, 2) for m in moves]}')
        return biggest > 1.0

    print('trials. Each starts from home so they cannot contaminate each other.')
    print()

    w = [wrap360(v) for v in plain]
    n = [nearest(v, angles(robot)[i]) for i, v in enumerate(plain)]
    # nearest() must be computed per the CURRENT reading, so recompute inside.
    res_w = trial('W: wrapped into [0,360)   <- what the driver sends', w)

    def trial_nearest():
        go_home()
        cur = angles(robot)
        tgt = [nearest(plain[i], cur[i]) for i in range(6)]
        before = angles(robot)
        try:
            robot.move_j(*tgt)
        except Exception as exc:  # noqa: BLE001
            print(f'  [FAIL] N: raised {type(exc).__name__}: {exc}')
            return None
        time.sleep(args.settle)
        after = angles(robot)
        moves = [after[i] - before[i] for i in range(6)]
        biggest = max(abs(m) for m in moves)
        tag = 'MOVE' if biggest > 1.0 else 'NOOP'
        print(f'  [{tag}] N: nearest whole-turn equivalent <- the proposed fix')
        print(f'         sent   {[round(v, 2) for v in tgt]}')
        print(f'         change {[round(m, 2) for m in moves]}')
        return biggest > 1.0

    res_n = trial_nearest()
    res_p = trial('P: plain, unwrapped    <- what the probes sent', plain)

    print()
    print('=' * 62)
    if res_p and not res_w:
        print('CONFIRMED: the device follows a 339.78 command literally and')
        print('the joint limit rejects it, so the driver\'s wrapping into')
        print('[0,360) is what makes every trajectory a silent no-op.')
        print()
        print('FIX: normalise each move_j target to the whole-turn equivalent')
        print('closest to the CURRENT pose, instead of into [0,360).')
    elif res_w and res_n and res_p:
        print('All three worked, so wrapping is not the problem here.')
        print('Re-examine what else differs between the driver and this script;')
        print('the most likely candidate is the target pose itself.')
    elif res_n and not res_w:
        print('The nearest-equivalent form works while the wrapped form does')
        print('not. Apply the nearest-equivalent normalisation.')
    else:
        print('See the individual results above.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
