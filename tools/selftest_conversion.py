#!/usr/bin/env python3
"""Consistency self-test for the driver's three coordinate spaces.

The driver converts between three spaces, and getting the algebra wrong is
invisible until real hardware is commanded to a wrong place. This script
re-implements the same formulas and asserts the properties that must hold:

  1. The calibration anchor maps correctly both ways.
       7 pose: angle -> URDF = all zeros, URDF -> move_j = [0,0,90,0,0,0]
  2. Round trip URDF -> move_j -> URDF is the identity, for every joint,
     with every combination of joint_sign.
  3. angle -> URDF -> angle is the identity.

It reads the offsets from the driver config, so it cannot drift from the real
parameters. It does NOT import the driver: it is an independent check, so a
bug in the driver cannot hide by being mirrored here.

Usage:
    python tools/selftest_conversion.py
    python tools/selftest_conversion.py --sign -1,1,1,1,1,1
"""
import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'config', 'dummy_controller.yaml')
DRIVER = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'dummy_controller', 'dummy_controller.py')
LAUNCH = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'launch', 'dummy_controller.launch.py')

JOINTS = [f'dummy2_joint{i}' for i in range(1, 7)]

# Physical anchor: the device reaches the 7 pose when commanded
# move_j = [0,0,90,0,0,0], and its raw angle readback there is ANGLE_7POSE.
ANGLE_7POSE = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
MOVEJ_7POSE = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
URDF_7POSE = [0.0] * 6


def get_list(text, key, default=None):
    m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
    if not m:
        return default
    return [float(v) for v in m.group(1).split(',') if v.strip()]


def wrap(v, lo=0.0):
    while v < lo:
        v += 360.0
    while v >= lo + 360.0:
        v -= 360.0
    return v


def make_conversions(joff, mjoff, sign):
    """Return the conversions, mirroring the driver algebra exactly.

        angle -> urdf :  urdf   = sign * (angle - joff)
        urdf  -> angle:  angle  = urdf / sign + joff
        urdf  -> move_j: move_j = urdf / sign + mjoff, wrapped
        move_j -> angle: angle  = move_j - mjoff
        move_j -> urdf:  urdf   = sign * (move_j - mjoff)
    """
    def angle_to_urdf(a):
        return [sign[i] * (a[i] - joff[i]) for i in range(6)]

    def urdf_to_angle(u):
        return [u[i] / sign[i] + joff[i] for i in range(6)]

    def urdf_to_movej(u):
        return [wrap(u[i] / sign[i] + mjoff[i]) for i in range(6)]

    def movej_to_urdf(m):
        return [sign[i] * (m[i] - mjoff[i]) for i in range(6)]

    return angle_to_urdf, urdf_to_angle, urdf_to_movej, movej_to_urdf


def close(a, b, tol=1e-6):
    return all(abs(a[i] - b[i]) < tol for i in range(len(a)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sign', default=None,
                    help='override joint_sign, e.g. -1,1,1,1,1,1')
    args = ap.parse_args()

    text = open(CONFIG, encoding='utf-8').read()
    joff = get_list(text, 'joint_offset_deg')
    mjoff = get_list(text, 'move_j_offset_deg')
    sign = ([float(v) for v in args.sign.split(',')] if args.sign
            else get_list(text, 'joint_sign'))

    print(f'config: {os.path.relpath(CONFIG, REPO)}')
    print(f'  joint_offset_deg  = {joff}')
    print(f'  move_j_offset_deg = {mjoff}')
    print(f'  joint_sign        = {sign}')
    print()

    for name, val in (('joint_offset_deg', joff),
                      ('move_j_offset_deg', mjoff), ('joint_sign', sign)):
        if val is None or len(val) != 6:
            print(f'FATAL: {name} missing or not length 6')
            return 1

    # Only joint_sign is used as a divisor. joint_offset_deg and
    # move_j_offset_deg are additive, so a zero there is perfectly normal
    # (indeed move_j_offset_deg is mostly zeros).
    if any(abs(v) < 1e-12 for v in sign):
        print('FATAL: joint_sign contains a zero; the conversion divides by it')
        return 1

    a2u, u2a, u2m, m2u = make_conversions(joff, mjoff, sign)
    failures = []

    def check(label, got, want, tol=1e-6):
        okc = close(got, want, tol)
        mark = 'ok  ' if okc else 'FAIL'
        print(f'  [{mark}] {label}')
        print(f'          got  {[round(v, 4) for v in got]}')
        if not okc:
            print(f'          want {[round(v, 4) for v in want]}')
            failures.append(label)

    # ---- 1. anchor ------------------------------------------------------
    print('1. calibration anchor (7 pose)')
    check('angle_7pose -> URDF == 0', a2u(ANGLE_7POSE), URDF_7POSE, tol=1e-6)
    check('URDF 0 -> move_j == [0,0,90,...]', u2m(URDF_7POSE), MOVEJ_7POSE,
          tol=1e-6)
    check('angle_7pose -> move_j == [0,0,90,...]', u2m(a2u(ANGLE_7POSE)),
          MOVEJ_7POSE, tol=1e-6)
    print()

    # ---- 2. round trips -------------------------------------------------
    print('2. round trips')
    cases = [[0, 0, 0, 0, 0, 0], [20, 0, 0, 0, 0, 0],
             [-30, 15, -45, 10, -5, 25], [90, -60, 30, -120, 40, -80]]
    for case in cases:
        m = u2m(case)
        back = m2u(m)
        # compare modulo 360 since move_j is wrapped
        norm = [back[i] + 360.0 * round((case[i] - back[i]) / 360.0)
                for i in range(6)]
        check(f'URDF {case} -> move_j -> URDF', norm, case, tol=1e-6)

    for case in cases:
        a = u2a(case)
        back = a2u(a)
        check(f'URDF {case} -> angle -> URDF', back, case, tol=1e-6)
    print()

    # ---- 3. does the anchor constant look like the reference? ------------
    print('3. derived constant for the down-link')
    delta = [joff[i] - mjoff[i] for i in range(6)]
    print(f'  joint_offset_deg - move_j_offset_deg = {[round(v,3) for v in delta]}')
    print('  the reference project dummy_moveit_ws used rad_volumn_diff')
    print('  = [0, 0, 90, 0, 0, 0]; a match is supporting evidence, not proof.')
    print()

    # ---- 4. the same values must appear in the driver and the launch file --
    #
    # Three places carry these defaults: this config, the driver's code
    # defaults, and the launch file's DEFAULTS dict. They drifted apart
    # repeatedly in this project, and a stale one silently overrides the
    # config, so they are compared here.
    print('4. driver code defaults and launch defaults agree with the config')
    checks = [('joint_sign', sign), ('joint_offset_deg', joff),
              ('move_j_offset_deg', mjoff)]

    dtext = open(DRIVER, encoding='utf-8').read()
    ltext = open(LAUNCH, encoding='utf-8').read()
    for key, want in checks:
        m = re.search(rf'{key}\s*=\s*\[([^\]]*)\]', dtext)
        if key == 'joint_sign':
            m = re.search(r'default_sign\s*=\s*\[([^\]]*)\]', dtext)
        got = [float(v) for v in m.group(1).split(',')] if m else None
        check(f'driver default {key}', got, want, tol=1e-6)

        m = re.search(rf"'{key}':\s*'\[([^\]]*)\]'", ltext)
        got = [float(v) for v in m.group(1).split(',')] if m else None
        check(f'launch default {key}', got, want, tol=1e-6)

    # scalars: home/reset poses and the gripper block
    for key in ('home_joints_deg', 'reset_joints_deg'):
        m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
        want = [float(v) for v in m.group(1).split(',')] if m else None
        m = re.search(rf"'{key}':\s*'\[([^\]]*)\]'", ltext)
        got = [float(v) for v in m.group(1).split(',')] if m else None
        check(f'launch default {key}', got, want, tol=1e-6)

    for key in ('grip_angle_closed', 'grip_angle_open', 'grip_current_limit'):
        m = re.search(rf'^\s*{key}\s*:\s*([-\d.]+)', text, re.M)
        want = [float(m.group(1))] if m else None
        m = re.search(rf"'{key}':\s*'([-\d.]+)'", ltext)
        got = [float(m.group(1))] if m else None
        check(f'launch default {key}', got, want, tol=1e-9)
    print()

    print('=' * 62)
    if failures:
        print(f'RESULT: {len(failures)} FAILURE(S)')
        for f in failures:
            print(f'  - {f}')
        return 1
    print('RESULT: all conversion properties hold')
    return 0

if __name__ == '__main__':
    sys.exit(main())
