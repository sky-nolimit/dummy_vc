#!/usr/bin/env python3
"""Check the arm-waiting logic against simulated devices, without hardware.

`wait_settled` decides whether a move happened, so a bug in it produces
confident wrong answers -- which is exactly what happened: an earlier version
returned on the first quiet window, and because this device has a start-up lag
of a second or two, it reported "the joint did not move" while the arm was
still stationary and about to start.

The cases below pin that down. Each simulates a device with a given lag and
speed and asserts what the waiter must conclude.

    python3 tools/selftest_wait.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rig_common import wait_settled  # noqa: E402

FAILURES = []


def check(label, got, want, tol=0.05):
    ok = (abs(got - want) <= tol if isinstance(want, float)
          else got == want)
    print(f'  [{"ok  " if ok else "FAIL"}] {label}: got {got!r}, want {want!r}')
    if not ok:
        FAILURES.append(label)
    return ok


def simulate(lag, travel_time, target=10.0, spin_time=0.15):
    """A device that waits `lag` s, moves over `travel_time` s, then stops.

    `travel_time = 0` means the device never moves at all: that is the shape of
    a dropped command, and it is what case 3 exercises.
    """
    t0 = time.monotonic()

    def spin(seconds):
        time.sleep(seconds)

    def read():
        t = time.monotonic() - t0
        if travel_time <= 0.0:
            return [0.0] * 6          # never moves
        if t <= lag:
            a = 0.0
        elif t <= lag + travel_time:
            a = (t - lag) / travel_time * target
        else:
            a = target
        return [a] * 6

    return wait_settled(spin, read, timeout=lag + travel_time + 6.0)


def main():
    print('=' * 72)
    print('1. a slow start must not be mistaken for "did not move"')
    print('=' * 72)
    print('  device waits 1.5 s, then takes 4 s to travel 10 deg.')
    print('  the buggy behaviour would return after ~0.6 s with moved=False.')
    r = simulate(lag=1.5, travel_time=4.0)
    check('reported moved', r.moved, True)
    check('final angle', r.angles[0], 10.0)
    check('did not time out', r.timed_out, False)
    print(f'  elapsed {r.elapsed:.1f} s (must exceed lag + travel = 5.5 s)')
    check('elapsed covers the whole move', r.elapsed > 5.5, True)
    print()

    print('=' * 72)
    print('2. a fast device is still detected and not waited out')
    print('=' * 72)
    print('  device starts immediately, travels in 0.5 s.')
    r = simulate(lag=0.0, travel_time=0.5)
    check('reported moved', r.moved, True)
    check('final angle', r.angles[0], 10.0)
    print(f'  elapsed {r.elapsed:.1f} s; it must not sit for the start lag')
    check('did not waste the start lag', r.elapsed < 4.0, True)
    print()

    print('=' * 72)
    print('3. a device that never moves is reported as not moving')
    print('=' * 72)
    print('  this is the real signal for "the device dropped the command"')
    r = simulate(lag=0.0, travel_time=0.0)
    check('reported moved', r.moved, False)
    check('final angle', r.angles[0], 0.0)
    print(f'  elapsed {r.elapsed:.1f} s (should be about the start lag)')
    print()

    print('=' * 72)
    print('4. a move longer than the timeout is flagged, not passed off')
    print('=' * 72)
    print('  the caller must be able to tell "still travelling" from "done"')
    t0 = time.monotonic()

    def spin(s):
        time.sleep(s)

    def read():
        # travels forever
        return [(time.monotonic() - t0) * 2.0] * 6

    r = wait_settled(spin, read, timeout=2.0)
    check('timed_out is set', r.timed_out, True)
    check('moved is set', r.moved, True)
    print()

    print('=' * 72)
    if FAILURES:
        print(f'RESULT: {len(FAILURES)} FAILURE(S)')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('RESULT: the waiting logic behaves correctly in all four cases')
    return 0


if __name__ == '__main__':
    sys.exit(main())
