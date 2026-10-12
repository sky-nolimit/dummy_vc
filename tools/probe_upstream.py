#!/usr/bin/env python3
"""Decisive test: reproduce the UPSTREAM driver's exact device session.

Context: the upstream driver (provided by the user) reportedly controls the arm
fine, while this project's driver and all probes see move_j silently do
nothing. A line-by-line comparison of the device calls found them identical --
same move_j, same set_enable, same set_rgb_mode, same arguments. So the fault
is not in the set of calls. Two candidate explanations remain:

  1. Something about the PROCESS state (the upstream was run with
     `ros2 run ... dummy_controller`, this project with a launch file). If so,
     replicating the upstream's exact session here will MOVE the arm.
  2. Argument TYPE. This project passes numpy.float64 values, because the
     trajectory path originates from np.array. The upstream also uses numpy
     arrays, but its joints_callback passes the message positions straight
     through. If fibre cannot serialise numpy scalars, move_j would become a
     silent no-op exactly as observed. This is cheap to test.

This script therefore tries the same target three ways and reports which, if
any, moves the arm:

    A. python floats            move_j(1.0, 2.0, ...)
    B. numpy scalars            move_j(np.float64(1.0), ...)
    C. numpy array unpacked     a = np.array([...]); move_j(a[0], a[1], ...)

It also replicates the upstream's startup order (find_any -> set_enable(True)
-> move_j(home)) with NOTHING else in between, which is what is believed to
work on this hardware.

SAFETY: moves the arm. Keep a hand on the power switch.

Run WITHOUT dummy_controller:

    pkill -f dummy_controller
    python3 tools/probe_upstream.py
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
HOME = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]      # upstream home_joints


def angles(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def main():
    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except ImportError as exc:
        print(f'FATAL: cannot import dummy_cli_tool ({exc})')
        return 2

    print('=== probe_upstream: replicate the upstream session exactly ===')
    print()

    # ---- REPLICA of upstream init_dummy(), nothing more -----------------
    print('--- upstream replica: find_any() -> set_enable(True) -> move_j(home)')
    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('    FATAL: no device found. Stop dummy_controller first.')
        return 1
    robot = dev.robot
    print(f'    connected; angles = {[round(v, 3) for v in angles(robot)]}')

    robot.set_enable(True)
    print('    set_enable(True) done')

    before = angles(robot)
    print(f'    sending move_j({HOME}) ...')
    try:
        robot.move_j(HOME[0], HOME[1], HOME[2], HOME[3], HOME[4], HOME[5])
    except Exception as exc:  # noqa: BLE001
        print(f'    move_j raised {type(exc).__name__}: {exc}')
    time.sleep(3.0)
    after = angles(robot)
    moved = max(abs(after[i] - before[i]) for i in range(6))
    print(f'    [{"MOVE" if moved > 1 else "NOOP"}] '
          f'before={[round(v, 3) for v in before]}')
    print(f'           after ={[round(v, 3) for v in after]}  '
          f'max |delta| = {moved:.3f} deg')
    if moved > 1:
        print()
        print('    => The upstream session WORKS here. The difference is not')
        print('       the device and not the call: it is this project\'s')
        print('       surrounding code or process. Compare on that basis.')
        return 0
    print()

    # ---- argument-type test --------------------------------------------
    print('--- argument type test (each preceded by homing for a clean start)')
    print()

    def trial(label, make_args):
        try:
            robot.homing()
            time.sleep(2.5)
        except Exception as exc:  # noqa: BLE001
            print(f'  [FAIL] {label}: homing raised {exc}')
            return
        base = angles(robot)
        target = list(base)
        target[0] = base[0] + 8.0        # 8 degrees on joint 1
        try:
            args = make_args(target)
        except Exception as exc:  # noqa: BLE001
            print(f'  [FAIL] {label}: building args raised {exc}')
            return
        types = ','.join(type(a).__name__ for a in args)
        try:
            robot.move_j(*args)
        except Exception as exc:  # noqa: BLE001
            print(f'  [FAIL] {label}: move_j raised {type(exc).__name__}: {exc}')
            return
        time.sleep(3.0)
        aft = angles(robot)
        d = max(abs(aft[i] - base[i]) for i in range(6))
        print(f'  [{"MOVE" if d > 1 else "NOOP"}] {label}')
        print(f'         arg types = {types}')
        print(f'         max |delta| = {d:.3f} deg')

    trial('A. plain python floats',
          lambda t: [float(v) for v in t])

    trial('B. numpy scalars (np.float64)',
          lambda t: [np.float64(v) for v in t])

    trial('C. numpy array, unpacked with indexing',
          lambda t: tuple(np.array(t)[i] for i in range(6)))

    print()
    print('=' * 62)
    print('How to read this:')
    print('  - If A moves and B/C do not, the bug is the numpy scalar types')
    print('    this project passes to move_j. Fix: cast to float in')
    print('    move_joints(), i.e. move_j(*[float(v) for v in joints_deg]).')
    print('  - If ALL of A/B/C move, then move_j works and the earlier NOOPs')
    print('    were a state problem (a preceding call left the device unable')
    print('    to accept motion). The upstream replica above tells you which.')
    print('  - If NONE move while homing works, the device is refusing')
    print('    absolute setpoints regardless of argument type; that is a')
    print('    firmware/mode issue and needs the vendor tool to confirm.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
