#!/usr/bin/env python3
"""Verify the three coordinate spaces of the Dummy driver are consistent.

The driver juggles three spaces and they are easy to mix up:

  move_j space   robot.move_j angles (degrees). home/reset live here.
  angle space    robot.joint_N.angle readback (degrees).
  URDF space     URDF / MoveIt / /joint_states (radians).

Relations:
  URDF  = sign * (angle - joint_offset_deg)
  angle = move_j - move_j_offset_deg

This script reads the offsets from the driver config, so it can never drift
from the real parameters, and checks:

  1. The known reference point: move_j j3=90  ->  angle 2.30  ->  URDF 0.
  2. The 7-pose readback maps to URDF all-zeros.
  3. Round-trip URDF -> move_j -> URDF is the identity.
  4. Config file and driver defaults agree.

Usage:
    python tools/check_coord_spaces.py
"""
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

# The empirical anchor that pins the whole chain down.
# Measured with the arm in the 7 pose (calibration node, dry_run read-only).
SEVEN_POSE_ANGLE = [0.03, 2.03, 2.30, -0.03, 0.18, -0.03]
SEVEN_POSE_MOVEJ = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
SEVEN_POSE_URDF = [0.0] * 6

failures = []


def fail(msg):
    failures.append(msg)
    print(f'  FAIL  {msg}')


def ok(msg):
    print(f'  ok    {msg}')


def get_list(text, key, default):
    m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
    if not m:
        return default
    return [float(v) for v in m.group(1).split(',') if v.strip()]


def main():
    text = open(CONFIG, encoding='utf-8').read()
    sign = get_list(text, 'joint_sign', None)
    off = get_list(text, 'joint_offset_deg', None)
    mjoff = get_list(text, 'move_j_offset_deg', None)

    print(f'config: {os.path.relpath(CONFIG, REPO)}')
    print(f'  joint_sign          = {sign}')
    print(f'  joint_offset_deg    = {off}')
    print(f'  move_j_offset_deg   = {mjoff}')
    print()

    for name, val in (('joint_sign', sign), ('joint_offset_deg', off),
                      ('move_j_offset_deg', mjoff)):
        if val is None or len(val) != 6:
            print(f'FATAL: {name} missing or not length 6 in the config')
            return 1

    # ---- the three conversions, mirroring the driver ---------------------
    def angle_to_urdf_deg(a):
        return [sign[i] * (a[i] - off[i]) for i in range(6)]

    def movej_to_angle(m):
        return [m[i] - mjoff[i] for i in range(6)]

    def urdf_deg_to_movej(u):
        out = []
        for i in range(6):
            s = sign[i] if abs(sign[i]) > 1e-12 else 1.0
            out.append(u[i] / s + off[i] + mjoff[i])
        return out

    def close(a, b, tol=1e-6):
        return all(abs(a[i] - b[i]) < tol for i in range(6))

    # ---- check 1: driver defaults match the config -----------------------
    print('1. config vs driver defaults')
    dtext = open(DRIVER, encoding='utf-8').read()
    d_sign = get_list(dtext, 'default_sign', None) or \
        get_list(dtext, 'joint_sign', None)
    m = re.search(r'default_offset_deg\s*=\s*\[([^\]]*)\]', dtext)
    d_off = [float(v) for v in m.group(1).split(',')] if m else None
    m = re.search(r'default_movej_offset_deg\s*=\s*\[([^\]]*)\]', dtext)
    d_mjoff = [float(v) for v in m.group(1).split(',')] if m else None

    if d_off is None or not close(d_off, off):
        fail(f'joint_offset_deg differs: driver={d_off} config={off}')
    else:
        ok('joint_offset_deg matches')
    if d_mjoff is None or not close(d_mjoff, mjoff):
        fail(f'move_j_offset_deg differs: driver={d_mjoff} config={mjoff}')
    else:
        ok('move_j_offset_deg matches')
    print()

    # ---- check 2: the empirical anchor -----------------------------------
    print('2. 7-pose anchor: move_j [0,0,90,..] -> angle -> URDF all zeros')
    a = movej_to_angle(SEVEN_POSE_MOVEJ)
    u = angle_to_urdf_deg(a)
    print(f'  move_j = {SEVEN_POSE_MOVEJ}')
    print(f'  -> angle = {[round(v, 3) for v in a]}')
    print(f'  -> URDF  = {[round(v, 4) for v in u]}')
    if not close(u, SEVEN_POSE_URDF, tol=3.0):
        fail('move_j 7-pose does not map to URDF zeros')
    else:
        ok('maps to URDF zeros (within 3 deg)')
    print()

    # ---- check 3: the measured readback ----------------------------------
    print('3. measured 7-pose readback -> URDF all zeros')
    u2 = angle_to_urdf_deg(SEVEN_POSE_ANGLE)
    print(f'  angle = {SEVEN_POSE_ANGLE}')
    print(f'  -> URDF = {[round(v, 4) for v in u2]}')
    if not close(u2, SEVEN_POSE_URDF, tol=1e-6):
        fail('measured 7-pose readback does not map to URDF zeros')
    else:
        ok('maps to URDF zeros exactly (this is how the offsets were derived)')
    print()

    # ---- check 4: round trip ---------------------------------------------
    print('4. round trip URDF -> move_j -> URDF')
    cases = [[0, 0, 0, 0, 0, 0], [0, 0, 90, 0, 0, 0],
             [10, -30, 45, 5, 20, -15], [-45, 20, -60, 12, -8, 33]]
    for case in cases:
        mj = urdf_deg_to_movej(case)
        back = angle_to_urdf_deg(movej_to_angle(mj))
        good = close(case, back, tol=1e-6)
        print(f'  URDF {[round(v,1) for v in case]}')
        print(f'    -> move_j {[round(v,2) for v in mj]}')
        print(f'    -> back   {[round(v,1) for v in back]}   '
              f'{"ok" if good else "MISMATCH"}')
        if not good:
            fail(f'round trip mismatch for {case}')
    print()

    # ---- check 5: limits -------------------------------------------------
    print('5. named poses within URDF limits')
    limits = {
        'dummy2_joint1': (-180, 180), 'dummy2_joint2': (-120, 75),
        'dummy2_joint3': (-90, 75), 'dummy2_joint4': (-180, 180),
        'dummy2_joint5': (-100, 100), 'dummy2_joint6': (-180, 180),
    }
    home = get_list(text, 'home_joints_deg', None)
    reset = get_list(text, 'reset_joints_deg', None)
    for label, pose in (('home', home), ('reset', reset)):
        if pose is None:
            fail(f'{label} not found in config')
            continue
        u = angle_to_urdf_deg(movej_to_angle(pose))
        bad = []
        for i, n in enumerate(sorted(limits)):
            lo, hi = limits[n]
            if u[i] < lo - 0.5 or u[i] > hi + 0.5:
                bad.append(f'{n}={u[i]:.1f}')
        print(f'  {label}: move_j {pose}')
        print(f'    -> URDF {[round(v,1) for v in u]}')
        if bad:
            print(f'    WARNING outside URDF limits: {", ".join(bad)}')
            print('      (the real arm may still reach it; URDF limits are')
            print('       inherited and not yet verified against hardware)')
        else:
            ok(f'{label} within limits')
    print()

    print('=' * 62)
    if failures:
        print(f'RESULT: {len(failures)} FAILURE(S)')
        for f in failures:
            print(f'  - {f}')
        return 1
    print('RESULT: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
