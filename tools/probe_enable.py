#!/usr/bin/env python3
"""Find the missing step that makes move_j actually move the arm.

Established by probe_interface.py on the real device:

    [NOOP] set_enable(True) then move_j        nothing moved
    [NOOP] move_j alone                        nothing moved
    [NOOP] joint_1.set_position                nothing moved
    [NOOP] joint_1.set_position_with_time      nothing moved
    [MOVE] robot.homing()                      moved 90 deg, perfectly

So the hardware, the power stage and the transport are all fine -- homing
drives the arm. move_j and set_position return success and do nothing, which is
the signature of a missing enable/mode step rather than a broken device.

Note the interface dump: BOTH robot and each joint expose set_enable(). The
gripper already needed its own hand.set_enable(True) (robot.set_enable did not
cover it), so the working hypothesis is that the joint axes need their own
enable too, and move_j silently no-ops while they are disabled.

This script tries one hypothesis at a time and reports the angle delta, so the
[MOVE] line names the missing call. It MOVES THE ARM: keep a hand on the power
switch. Each trial starts from a freshly homed pose so the trials do not
interfere.

Run WITHOUT dummy_controller:

    pkill -f dummy_controller
    python3 tools/probe_enable.py
    python3 tools/probe_enable.py --delta 10
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


def angles(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delta', type=float, default=8.0,
                    help='degrees to nudge joint_1 by (default 8)')
    ap.add_argument('--timeout', type=float, default=15.0)
    ap.add_argument('--settle', type=float, default=3.0)
    args = ap.parse_args()

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except ImportError as exc:
        print(f'FATAL: cannot import dummy_cli_tool ({exc})')
        return 2

    dev = ref_tool.find_any(path='usb', timeout=args.timeout)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first '
              '(only one process can hold the fibre link).')
        return 1
    robot = dev.robot
    print('=== probe_enable: which call makes move_j take effect? ===')
    print()

    results = []

    def trial(label, setup):
        """Home, apply `setup`, then move_j and measure.

        Homing first is deliberate: it both verifies the arm can move at all
        right now and gives every trial the same starting pose, so a movement
        cannot be mistaken for drift from the previous trial.
        """
        print(f'--- {label}')
        try:
            print('    homing to reset ...')
            robot.homing()
            time.sleep(args.settle)
        except Exception as exc:  # noqa: BLE001
            print(f'    [FAIL] homing raised {type(exc).__name__}: {exc}')
            results.append((label, None))
            print()
            return

        base = angles(robot)
        print(f'    homed angles = {[round(v, 2) for v in base]}')

        try:
            setup()
            print('    setup applied')
        except Exception as exc:  # noqa: BLE001
            print(f'    [FAIL] setup raised {type(exc).__name__}: {exc}')
            results.append((label, None))
            print()
            return

        target = list(base)
        target[0] = base[0] + args.delta
        print(f'    move_j(joint_1 {base[0]:.2f} -> {target[0]:.2f}) ...')
        try:
            robot.move_j(*target)
        except Exception as exc:  # noqa: BLE001
            print(f'    [FAIL] move_j raised {type(exc).__name__}: {exc}')
            results.append((label, None))
            print()
            return

        time.sleep(args.settle)
        after = angles(robot)
        moves = [after[i] - base[i] for i in range(6)]
        biggest = max(abs(m) for m in moves)
        verdict = 'MOVE' if biggest > 1.0 else 'NOOP'
        print(f'    [{verdict}] after = {[round(v, 2) for v in after]}')
        print(f'           per-joint change = {[round(m, 2) for m in moves]}')
        results.append((label, verdict))
        print()

    # Hypothesis 1: nothing extra (this is what the driver already does).
    trial('move_j alone, after homing', lambda: None)

    # Hypothesis 2: the joint axes need their own enable, like the gripper did.
    def enable_joints():
        for n in JR:
            getattr(robot, n).set_enable(True)
    trial('enable every joint, then move_j', enable_joints)

    # Hypothesis 3: both robot and joints enabled.
    trial('robot.set_enable + every joint, then move_j',
          lambda: (robot.set_enable(True), enable_joints()))

    # Hypothesis 4: someone else holds the axis; try disabling first, then
    # enabling, in case the axis is latched in a non-position mode.
    def reenable_joints():
        for n in JR:
            getattr(robot, n).set_enable(False)
        time.sleep(0.3)
        robot.set_enable(True)
        for n in JR:
            getattr(robot, n).set_enable(True)
    trial('disable then re-enable all, then move_j', reenable_joints)

    # Hypothesis 5: a zero-velocity or zero-current leftover. Set explicit
    # limits before moving, in case a limit of 0 is silently clamping motion.
    def set_limits():
        for n in JR:
            getattr(robot, n).set_velocity_limit(10.0)
            getattr(robot, n).set_current_limit(5.0)
    trial('set velocity/current limits, then move_j', set_limits)

    print('=' * 62)
    movers = [lab for lab, v in results if v == 'MOVE']
    if movers:
        print('The call that makes move_j work:')
        for lab in movers:
            print(f'  - {lab}')
        print()
        print('Any setup listed there must be added to the driver BEFORE the')
        print('first move_j (i.e. in init_dummy or on_configure), otherwise')
        print('every trajectory silently does nothing.')
    else:
        print('RESULT: nothing made move_j move the arm, even though homing')
        print('        works. That is unusual: it means the device accepts')
        print('        homing but refuses absolute joint commands. Next step')
        print('        is to compare against the vendor tool / firmware docs,')
        print('        since the code path itself is proven to reach the device.')
    return 0 if movers else 1


if __name__ == '__main__':
    sys.exit(main())
