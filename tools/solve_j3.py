#!/usr/bin/env python3
"""Work out what j3's calibration must be, given the measurement.

The user ran tools/probe_mapping.py --joint 3 and got a clean line:

    sent     readback    readback - sent
    60.00   -120.023        -180.023
    75.00   -104.995        -179.995
    90.00    -89.997        -179.997
   105.00    -74.989        -179.989
   120.00    -59.980        -179.980

least squares: gain +1.0006, offset -180.052, residuals under 0.011 deg.

So on this joint the device behaves as  readback = move_j - 180  with a gain of
+1. This script asks what that implies for the two calibration parameters, and
shows that the values currently in the config satisfy neither.

Usage:
    python tools/solve_j3.py
"""
import math

# measured
SAMPLES = [(60.0, -120.023), (75.0, -104.995), (90.0, -89.997),
           (105.0, -74.989), (120.0, -59.980)]

# from the config
CONFIG_SIGN = -1.0
CONFIG_JOFF = -91.474
CONFIG_MJOFF = 90.0

# two independent observations of the same physical pose
RAW_7POSE = -91.474          # readback at the 7 pose
HOME_MOVEJ = 90.0            # home_joints_deg says move_j j3 = 90 at home
HOME_RAW = -90.0             # homing() leaves the readback at -90

# driver chain
#   raw    = urdf/sign + joff
#   move_j = raw + mjoff
#   so     raw = (move_j - mjoff)/sign + joff


def fit():
    n = len(SAMPLES)
    sx = sum(s for s, _ in SAMPLES)
    sy = sum(r for _, r in SAMPLES)
    sxx = sum(s * s for s, _ in SAMPLES)
    sxy = sum(s * r for s, r in SAMPLES)
    gain = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    offset = (sy - gain * sx) / n
    resid = [r - (gain * s + offset) for s, r in SAMPLES]
    return gain, offset, resid


def main():
    gain, offset, resid = fit()
    print('=' * 70)
    print('1. WHAT THE DEVICE DOES (measured)')
    print('=' * 70)
    print(f'  readback = {gain:+.4f} * move_j {offset:+.3f}')
    print(f'  residuals = {[round(r, 4) for r in resid]}')
    print(f'  => gain +1, offset -180. Same quantity, 180 deg apart.')
    print()

    print('=' * 70)
    print('2. THE TWO INDEPENDENT POSE OBSERVATIONS')
    print('=' * 70)
    print(f'  homing(): readback {HOME_RAW:+.1f}, and home_joints_deg says')
    print(f'            move_j should be {HOME_MOVEJ:+.1f} there.')
    print(f'            check against the measurement: '
          f'{HOME_MOVEJ:+.1f} - 180 = {HOME_MOVEJ - 180:+.1f}  '
          f'vs {HOME_RAW:+.1f}')
    implied = RAW_7POSE + 180.0
    print(f'  7 pose : readback {RAW_7POSE:+.3f}')
    print(f'            by the measurement that means move_j = '
          f'{RAW_7POSE:+.3f} + 180 = {implied:+.3f}')
    print(f'            so the 7 pose sits at move_j j3 = {implied:.1f},')
    print(f'            not at 90 and not at -1.47.')
    print()

    print('=' * 70)
    print('3. WHAT THE CONFIG IMPLIES, AND WHY IT CANNOT BE RIGHT')
    print('=' * 70)
    print(f'  config joint_sign[3] = {CONFIG_SIGN:+.0f}, '
          f'joint_offset_deg[3] = {CONFIG_JOFF:+.3f}, '
          f'move_j_offset_deg[3] = {CONFIG_MJOFF:+.1f}')
    print()
    print('  the driver would then compute, at move_j = 90:')
    s = CONFIG_SIGN
    raw_at_90 = (HOME_MOVEJ - CONFIG_MJOFF) / s + CONFIG_JOFF
    print(f'    raw = (90 - {CONFIG_MJOFF:.0f})/{s:+.0f} + {CONFIG_JOFF:.3f}'
          f' = {raw_at_90:+.3f}')
    print(f'    but the device actually reports {HOME_RAW:+.1f} there.')
    print(f'    that is a {abs(raw_at_90 - HOME_RAW):.1f} deg error.')
    print()

    print('=' * 70)
    print('4. WHAT THE OFFSET WOULD HAVE TO BE, FOR EACH SIGN')
    print('=' * 70)
    print('  Require raw = move_j - 180, using the 7 pose as the reference')
    print(f'  point (move_j = {implied:.3f}, raw = {RAW_7POSE:+.3f}):')
    print()
    print(f'  {"sign":>6} {"required joint_offset_deg[3]":>32}')
    for sign in (1.0, -1.0):
        joff = RAW_7POSE - (implied - CONFIG_MJOFF) / sign
        print(f'  {sign:>+6.0f} {joff:>32.3f}')
    print()
    print('  So the joint_offset_deg currently in the config (-91.474) is')
    print('  wrong either way: it should be about -1.5 with sign +1, or about')
    print('  178.5 with sign -1.')
    print()

    print('=' * 70)
    print('5. WHY THE SIGN CANNOT BE SETTLED HERE')
    print('=' * 70)
    print('  The sign says whether a positive move_j turns the joint the same')
    print('  way as a positive URDF angle. That is a physical fact about the')
    print('  assembly, not something derivable from these numbers -- both sign')
    print('  choices above can be made to fit the readback by picking the')
    print('  offset accordingly.')
    print()
    print('  The two candidates give completely different physical behaviour:')
    print('    sign +1: commanding a larger move_j turns the joint one way')
    print('    sign -1: it turns the other way, with the offset absorbing 180')
    print('  so they must be told apart by watching the arm, not by algebra.')
    return 0


if __name__ == '__main__':
    main()
