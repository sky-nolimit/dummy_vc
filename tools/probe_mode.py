#!/usr/bin/env python3
"""Find which command mode makes move_j act, and prove the two commands differ.

What is established on the real device:

    robot.homing()        moves the arm, perfectly, every time
    robot.move_j(...)     returns ok, changes nothing, in every enable state
    set_enable(True)      does energise the axes (arm resists a manual push)
    probe_attrs found     robot.set_command_mode, set_joint_speed,
                          set_joint_acc, resting, calibrate_home_offset, reboot

So the axes ARE energised and homing IS able to drive them, yet setpoint
commands do nothing. The untested difference is the command mode: homing is a
firmware routine that works regardless, while move_j is a setpoint command the
firmware may be ignoring in the current mode.

The reference driver that worked on this same arm never calls
set_command_mode, so it is not strictly required -- but if homing left the
device in a non-setpoint mode, that would explain everything.

This script:
  1. decodes the device's own protocol JSON (dev._json_data) and prints the
     declaration of set_command_mode and every motion endpoint, so argument
     names and valid values come from the firmware rather than a guess;
  2. tries each command mode in turn and reports whether move_j then works.

SAFETY: it MOVES THE ARM. Keep a hand on the power switch.

    pkill -f dummy_controller
    python3 tools/probe_mode.py
    python3 tools/probe_mode.py --max-mode 4
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
WANT = ('mode', 'move', 'enable', 'home', 'rest', 'calibrat', 'speed', 'acc')


def walk(node, path=''):
    if isinstance(node, dict):
        if 'name' in node and 'type' in node:
            yield path, node.get('name', ''), node
        for k, v in node.items():
            yield from walk(v, f'{path}.{k}' if path else k)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from walk(v, f'{path}[{i}]')


def angles(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-mode', type=int, default=5,
                    help='highest command-mode value to try (default 5)')
    ap.add_argument('--delta', type=float, default=8.0)
    ap.add_argument('--settle', type=float, default=2.5)
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
    print('=== probe_mode ===')
    print()

    # ---- 1. the firmware's own declaration of these endpoints -----------
    print('1. protocol declarations for the motion-related endpoints')
    raw = getattr(dev, '_json_data', None)
    if raw is None:
        print('   (dev._json_data missing; skipping)')
    else:
        try:
            doc = json.loads(raw) if isinstance(raw, str) else raw
            shown = 0
            for path, name, d in walk(doc):
                if any(k in name.lower() for k in WANT):
                    print(f'   --- {path}')
                    print('   ' + json.dumps(d, ensure_ascii=False)[:500])
                    shown += 1
            if not shown:
                print('   (no match; run tools/dump_protocol.py --raw to see it)')
        except (TypeError, ValueError) as exc:
            print(f'   (could not parse: {exc})')
    print()

    # ---- 2. does set_command_mode exist, and what does it take? --------
    print('2. calling set_command_mode with each value and retrying move_j')
    print()
    if not hasattr(robot, 'set_command_mode'):
        print('   set_command_mode not present on this device')
        return 1

    for mode in range(0, args.max_mode + 1):
        print(f'--- command mode {mode}')
        try:
            robot.homing()
            time.sleep(args.settle)
        except Exception as exc:  # noqa: BLE001
            print(f'    [FAIL] homing raised: {exc}')
            break
        base = angles(robot)
        print(f'    homed = {[round(v, 2) for v in base]}')

        try:
            robot.set_command_mode(mode)
            print(f'    set_command_mode({mode}) ok')
        except Exception as exc:  # noqa: BLE001
            print(f'    [FAIL] set_command_mode({mode}) raised '
                  f'{type(exc).__name__}: {exc}')
            print('           continuing to the next value')
            print()
            continue

        target = list(base)
        target[0] = base[0] + args.delta
        try:
            robot.move_j(*target)
        except Exception as exc:  # noqa: BLE001
            print(f'    [FAIL] move_j raised {type(exc).__name__}: {exc}')
            print()
            continue

        time.sleep(args.settle)
        after = angles(robot)
        moves = [after[i] - base[i] for i in range(6)]
        biggest = max(abs(m) for m in moves)
        if biggest > 1.0:
            print(f'    [MOVE] mode {mode} WORKS: change = '
                  f'{[round(m, 2) for m in moves]}')
        else:
            print(f'    [NOOP] max |delta| = {biggest:.3f} deg')
        print()

    print('=' * 62)
    print('If some mode shows [MOVE], that is the call the driver must make')
    print('before its first move_j (add it to init_dummy, right after')
    print('set_enable). If every mode is [NOOP], then move_j is not the path')
    print('this firmware uses for setpoints and we should follow the reference')
    print('driver more literally: set_enable(1) then move_j immediately, with')
    print('no homing() call in between.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
