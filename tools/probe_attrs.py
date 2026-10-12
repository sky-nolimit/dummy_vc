#!/usr/bin/env python3
"""Dump EVERYTHING the device exposes, plus whether it holds torque.

The earlier probe only looked for attribute names I had guessed. That can miss
the one call that matters. This lists the complete attribute set of the robot,
of one joint, and of the hand -- names only, no guessing -- and it reports
whether the arm stiffens when commanded to enable, which distinguishes
"torque on" from "silently ignored".

Why torque matters: robot.homing() moves the arm perfectly while every
move_j/set_position is a no-op. Those differ in one obvious way -- homing is a
firmware routine that can enable and mode its own axes, whereas a setpoint
command needs the axis already enabled and in position mode. If set_enable()
does not actually turn torque on, move_j writes into a disabled axis and
nothing happens. This script tests exactly that, separately from move_j.

Run WITHOUT dummy_controller:

    pkill -f dummy_controller
    python3 tools/probe_attrs.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


def names_of(obj):
    out = []
    for n in dir(obj):
        if n.startswith('__'):
            continue
        try:
            v = getattr(obj, n)
        except Exception:  # noqa: BLE001
            out.append((n, '<unreadable>'))
            continue
        if callable(v):
            out.append((n, 'callable'))
        else:
            out.append((n, repr(v)[:60]))
    return out


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
    j1 = robot.joint_1

    print('=== probe_attrs: complete attribute dump ===')
    print()
    print('--- dev (top level)')
    for n, v in names_of(dev):
        print(f'    {n}  [{v}]')
    print()

    print('--- robot  (ALL attributes, not a guessed list)')
    for n, v in names_of(robot):
        print(f'    {n}  [{v}]')
    print()

    print('--- robot.joint_1  (ALL attributes)')
    for n, v in names_of(j1):
        print(f'    {n}  [{v}]')
    print()

    hand = getattr(robot, 'hand', None)
    if hand is not None:
        print('--- robot.hand  (ALL attributes)')
        for n, v in names_of(hand):
            print(f'    {n}  [{v}]')
        print()

    print('--- per-joint angle right now')
    print(f'    {[round(float(getattr(robot, n).angle), 3) for n in JR]}')
    print()

    # ---- torque test, independent of any setpoint command ---------------
    print('=== torque test ===')
    print('This tells us whether set_enable actually energises the axes,')
    print('without involving move_j at all.')
    print()
    print('Sequence: read -> set_enable(False) -> read -> set_enable(True) -> read')
    print('A holding axis resists a push; if set_enable(True) does nothing,')
    print('the joint angles will drift freely and no setpoint will take effect.')
    print()

    for label, fn in (
        ('before any enable', None),
        ('after robot.set_enable(False)',
         lambda: robot.set_enable(False)),
        ('after robot.set_enable(True)',
         lambda: robot.set_enable(True)),
        ('after joint_1.set_enable(True)',
         lambda: j1.set_enable(True)),
    ):
        if fn is not None:
            try:
                fn()
            except Exception as exc:  # noqa: BLE001
                print(f'  {label}: raised {type(exc).__name__}: {exc}')
                continue
        time.sleep(1.0)
        vals = [float(getattr(robot, n).angle) for n in JR]
        print(f'  {label:<34} angles = {[round(v, 3) for v in vals]}')

    print()
    print('=' * 62)
    print('Read the attribute dumps above for a mode/setpoint method we have')
    print('not tried (anything with "mode", "setpoint", "target", "goto",')
    print('"erasing", "calibrat", "clear", "reset", "fault" in the name).')
    print()
    print('Then: with the arm enabled, try to move joint_1 BY HAND. If it')
    print('moves freely, torque is NOT on and that is the whole problem --')
    print('move_j will never do anything until the axis is actually energised.')
    print('Reference: homing() moves the arm, so the power stage works; the')
    print('question is purely whether set_enable reaches the axes.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
