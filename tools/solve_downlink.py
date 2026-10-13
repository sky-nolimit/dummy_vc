#!/usr/bin/env python3
"""Find the down-link formula that works for EVERY joint, from measured data.

The driver computes

    move_j = urdf/sign + joint_offset_deg + move_j_offset_deg

Measured against the two known poses it gives:

    joint_2:  74.572  wanted    0.000   off by +74.572 = +joint_offset_deg[1]
    joint_3:  -1.474  wanted   90.000   off by -91.474 = +joint_offset_deg[2]

so the error is exactly `joint_offset_deg` in both cases, which suggests the
term's sign is wrong. The candidate fix is

    move_j = urdf/sign + abs(joint_offset_deg) + move_j_offset_deg

with abs() because the offset's raw direction is what has to be added, while
the signed value is what the read chain subtracts.

This script checks the candidate against every measurement we have, and checks
the results land inside the accepted move_j intervals -- the thing that
actually matters, because a value outside its interval is dropped silently.

Usage:
    python tools/solve_downlink.py
"""
import argparse

K = [0.0, 75.0, -180.0, 0.0, 0.0, 0.0]
RANGES = [(-165.0, 165.0), (-75.0, 90.0), (45.0, 180.0),
          (-180.0, 165.0), (-90.0, 90.0), (-180.0, 180.0)]

SIGN = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
JOFF = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
MJOFF = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

HOME_CMD = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
HOME_RAW = [-0.000, 75.032, -90.016, -0.002, -0.000, -0.007]
RESET_CMD = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]
RESET_RAW = [-0.000, 0.04, -0.03, 0.001, 0.001, -0.001]


def fold(value, lo, hi):
    width = hi - lo
    x = (value - lo) % 360.0
    for c in (x - 360.0, x, x + 360.0):
        if -1e-9 <= c <= width + 1e-9:
            return lo + c
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sign', default=None,
                    help='comma separated, overrides the config value')
    args = ap.parse_args()
    sign = ([float(v) for v in args.sign.split(',')] if args.sign
            else list(SIGN))

    def read_chain(raw):
        """raw -> urdf, the formula the model uses."""
        return [sign[i] * (raw[i] - JOFF[i]) for i in range(6)]

    def old_down(urdf):
        return [urdf[i] / sign[i] + JOFF[i] + MJOFF[i] for i in range(6)]

    def new_down(urdf):
        return [urdf[i] / sign[i] + abs(JOFF[i]) + MJOFF[i] for i in range(6)]

    def measured_down(urdf):
        """Built straight from the measured raw = move_j + K relation.

        urdf = sign * (raw - joff)  =>  raw = urdf/sign + joff
        raw  = move_j + K           =>  move_j = raw - K
        """
        return [urdf[i] / sign[i] + JOFF[i] - K[i] for i in range(6)]

    print('=' * 74)
    print('MEASURED DEVICE RELATION')
    print('=' * 74)
    print(f'  raw = move_j + K,  K = {K}')
    print('  (home and reset agree on K, so this is measured, not fitted)')
    print()

    urdf_home = [0.0] * 6
    urdf_reset = read_chain(RESET_RAW)

    print('=' * 74)
    print('THE THREE CANDIDATE DOWN-LINKS, AT BOTH KNOWN POSES')
    print('=' * 74)
    for name, fn in (('current  urdf/sign + joff + mjoff', old_down),
                     ('candidate urdf/sign + |joff| + mjoff', new_down),
                     ('measured  urdf/sign + joff - K', measured_down)):
        print(f'  {name}')
        for label, urdf, want in (('home ', urdf_home, HOME_CMD),
                                  ('reset', urdf_reset, RESET_CMD)):
            got = fn(urdf)
            bad = [i + 1 for i in range(6) if abs(got[i] - want[i]) > 0.6]
            print(f'    {label}: {[round(v, 2) for v in got]}  '
                  f'wrong at {bad if bad else "none"}')
        print()

    print('=' * 74)
    print('DO THEY LAND INSIDE THE ACCEPTED INTERVALS?')
    print('=' * 74)
    print('  This is the test that matters: a value outside its interval is')
    print('  dropped SILENTLY by the device, taking the whole command with it.')
    print()
    for name, fn in (('current', old_down),
                     ('candidate', new_down),
                     ('measured', measured_down)):
        got = fn(urdf_home)
        bad = []
        for i in range(6):
            lo, hi = RANGES[i]
            if not (lo - 1e-6 <= got[i] <= hi + 1e-6):
                f = fold(got[i], lo, hi)
                bad.append((i + 1, round(got[i], 2), (lo, hi),
                            None if f is None else round(f, 2)))
        print(f'  {name:<10} move_j(urdf=0) = {[round(v, 2) for v in got]}')
        if bad:
            for j, v, rng, f in bad:
                print(f'      joint_{j} = {v} OUTSIDE {rng}'
                      + (f'  (fold gives {f})' if f is not None else
                         '  (no equivalent inside)'))
        else:
            print('      all inside their intervals')
    print()

    print('=' * 74)
    print('ROUND TRIP: does the model come back to what it started at?')
    print('=' * 74)
    print('  The two chains must be inverses. Feed the raw readback through')
    print('  the read chain, then through the down-link, and the move_j value')
    print('  that comes out must reproduce that raw via raw = move_j + K.')
    for name, fn in (('current', old_down), ('candidate', new_down),
                     ('measured', measured_down)):
        worst = 0.0
        for raw in (HOME_RAW, RESET_RAW):
            mj = fn(read_chain(raw))
            back = [mj[i] + K[i] for i in range(6)]
            worst = max(worst, max(abs(back[i] - raw[i]) for i in range(6)))
        verdict = 'consistent' if worst < 1.0 else f'BROKEN (off by {worst:.1f})'
        print(f'  {name:<10} worst raw mismatch = {worst:8.3f} deg   {verdict}')
    print()

    print('=' * 74)
    print('VERDICT')
    print('=' * 74)
    print('  Only the formulation that (a) reproduces both measured poses,')
    print('  (b) lands inside every accepted interval, and (c) round-trips,')
    print('  is usable. Use the one that passes all three.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
