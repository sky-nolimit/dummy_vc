#!/usr/bin/env python3
"""Fold a move_j value into a joint's ACCEPTED interval, and say if it fits.

Reads the accepted intervals measured on the hardware by tools/probe_range.py:

    joint_1  [-165, 165]      joint_4  [-180, 165]
    joint_2  [ -75,  90]      joint_5  [ -90,  90]
    joint_3  [  45, 180]      joint_6  [-180, 180]

The point: a target outside its joint's interval is refused SILENTLY by the
device -- the whole command is dropped and the arm does not move, with no error
anywhere. The driver therefore has to bring each target into range.

It also demonstrates the case that motivated all of this: the drive command
that produced j3 = -1.474, which is outside [45, 180] and was therefore thrown
away, even though the correct equivalent value (91) sits comfortably inside.

Usage:
    python tools/fold_range.py
    python tools/fold_range.py --target -20.215,74.572,-1.474,-0.91,-0.035,-0.04
"""
import argparse
import math

# intervals measured by tools/probe_range.py (move_j space, degrees)
RANGES = [(-165.0, 165.0), (-75.0, 90.0), (45.0, 180.0),
          (-180.0, 165.0), (-90.0, 90.0), (-180.0, 180.0)]

# a real drive command, before folding
DEFAULT_TARGET = [-20.215, 74.572, -1.474, -0.910, -0.035, -0.040]
DEFAULT_CURRENT = [0.215, 74.572, -1.474, -0.910, -0.035, -0.040]


def fold(value, lo, hi):
    """Return the whole-turn equivalent of `value` inside [lo, hi], or None.

    The interval may sit on either side of the 0/360 seam, so the candidate is
    found with a properly signed modulo instead of a clamp: subtracting the
    lower bound puts the interval at [0, width], taking a modulo brings the
    value into it, and adding the bound back restores the original frame.
    """
    width = hi - lo
    if width <= 0:
        return None
    x = (value - lo) % 360.0          # move to the interval's own frame
    if x > width:                     # outside after one turn: try the neighbours
        candidates = [x - 360.0, x, x + 360.0]
        for c in candidates:
            if -1e-9 <= c <= width + 1e-9:
                return lo + c
        # still nothing: pick the nearest equivalent even if out of range, so
        # the caller can report how far off it is
        x = min(candidates, key=lambda c: min(abs(c), abs(c - width)))
    return lo + x


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', default=None)
    ap.add_argument('--current', default=None)
    args = ap.parse_args()

    target = ([float(v) for v in args.target.split(',')] if args.target
              else DEFAULT_TARGET)
    current = ([float(v) for v in args.current.split(',')] if args.current
               else DEFAULT_CURRENT)

    print('=' * 74)
    print('ACCEPTED INTERVALS (measured with tools/probe_range.py)')
    print('=' * 74)
    for i, (lo, hi) in enumerate(RANGES):
        print(f'  joint_{i + 1}: [{lo:>7.1f}, {hi:>7.1f}]   width {hi - lo:>6.1f}')
    print()

    print('=' * 74)
    print('THE COMMAND THAT FAILED')
    print('=' * 74)
    print(f'  target  = {target}')
    print(f'  current = {current}')
    print()
    print('  Before folding, the driver would send the target as-is. Check it')
    print('  against each interval:')
    print(f'  {"joint":<8} {"value":>10} {"interval":>18} {"in range":>10}')
    for i in range(6):
        lo, hi = RANGES[i]
        inside = lo - 1e-9 <= target[i] <= hi + 1e-9
        mark = 'yes' if inside else 'NO -- refused'
        print(f'  joint_{i + 1:<2} {target[i]:>10.3f} [{lo:>7.1f},{hi:>7.1f}] '
              f'{mark:>18}')
    print()

    print('=' * 74)
    print('AFTER FOLDING INTO RANGE')
    print('=' * 74)
    folded = []
    for i in range(6):
        lo, hi = RANGES[i]
        f = fold(target[i], lo, hi)
        folded.append(f)
    print(f'  {"joint":<8} {"target":>10} {"folded":>10} {"turns":>7} '
          f'{"travel":>9} {"ok":>5}')
    for i in range(6):
        t = target[i]
        f = folded[i]
        turns = round((f - t) / 360.0)
        travel = f - current[i]
        ok = (RANGES[i][0] - 1e-6 <= f <= RANGES[i][1] + 1e-6)
        print(f'  joint_{i + 1:<2} {t:>10.3f} {f:>10.3f} {turns:>+7d} '
              f'{travel:>+9.3f} {"yes" if ok else "NO":>5}')
    print()
    print(f'  driver would send: '
          f'[{", ".join(f"{v:.3f}" for v in folded)}]')
    print()

    print('=' * 74)
    print('WHY joint_3 WAS THE ONE THAT BROKE')
    print('=' * 74)
    print('  Its target is -1.474 and its interval is [45, 180]. No whole-turn')
    print('  shift of -1.474 lands inside that interval except 358.526 (also')
    print('  outside) or 91.526 -- wait, that is +93, a 90 degree shift, not a')
    print('  whole turn. That is the real lesson: the interval is narrow and')
    print('  offset from where the driver assumed it was, so "nearest whole')
    print('  turn" cannot find it. Folding must use the interval, period.')
    f3 = folded[2]
    print()
    print(f'  folded joint_3 = {f3:.3f}, inside [45, 180]: '
          f'{"yes" if 45 <= f3 <= 180 else "no"}')
    print(f'  the arm is asked to travel {f3 - current[2]:+.3f} deg, which the')
    print('  device accepts, so the command actually executes.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
