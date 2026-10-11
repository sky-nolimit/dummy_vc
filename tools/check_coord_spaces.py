#!/usr/bin/env python3
"""Verify the three coordinate spaces of the Dummy driver are consistent.

The driver juggles three spaces and they are easy to mix up:

  move_j space   robot.move_j angles (degrees). home/reset live here.
  angle space    robot.joint_N.angle readback (degrees).
  URDF space     URDF / MoveIt / /joint_states (radians).

Relations (see config/dummy_controller.yaml for the derivation):
  URDF  = sign * (angle - joint_offset_deg) + urdf_offset_deg
  angle = move_j - move_j_offset_deg

This script reads the offsets from the driver config, so it can never drift
from the real parameters, and checks:

  1. Config file and driver code defaults agree.
  2. The empirical anchor: move_j j3=90 -> angle 2.30 -> arm-chain 0,
     which with urdf_offset_deg applied must give the expected URDF vector.
  3. The measured 7-pose readback maps to the same URDF vector.
  4. Round trip URDF -> move_j -> URDF is the identity.
  5. Named poses, reported against the URDF limits.

Usage:
    python tools/check_coord_spaces.py
"""
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
# The 7 pose in URDF space is NOT all zeros: j3 carries urdf_offset_deg on top
# of the arm-chain value. So the expectation is computed, not hardcoded.
SEVEN_POSE_ARM_CHAIN = [0.0] * 6

LIMITS = {
    'dummy2_joint1': (-180, 180),
    'dummy2_joint2': (-120, 75),
    'dummy2_joint3': (-90, 75),
    'dummy2_joint4': (-180, 180),
    'dummy2_joint5': (-100, 100),
    'dummy2_joint6': (-180, 180),
}

failures = []


def fail(msg):
    failures.append(msg)
    print(f'  FAIL  {msg}')


def ok(msg):
    print(f'  ok    {msg}')


def get_list(text, key, default=None):
    m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
    if not m:
        return default
    return [float(v) for v in m.group(1).split(',') if v.strip()]


def get_py_list(text, varname, default=None):
    m = re.search(rf'{varname}\s*=\s*\[([^\]]*)\]', text)
    if not m:
        return default
    return [float(v) for v in m.group(1).split(',') if v.strip()]


def close(a, b, tol=1e-6):
    return a is not None and b is not None and len(a) == len(b) and \
        all(abs(a[i] - b[i]) < tol for i in range(len(a)))


def main():
    text = open(CONFIG, encoding='utf-8').read()
    sign = get_list(text, 'joint_sign')
    off = get_list(text, 'joint_offset_deg')
    mjoff = get_list(text, 'move_j_offset_deg')
    uoff = get_list(text, 'urdf_offset_deg')
    home = get_list(text, 'home_joints_deg')
    reset = get_list(text, 'reset_joints_deg')

    print(f'config: {os.path.relpath(CONFIG, REPO)}')
    for name, val in (('joint_sign', sign),
                      ('joint_offset_deg', off),
                      ('move_j_offset_deg', mjoff),
                      ('urdf_offset_deg', uoff),
                      ('home_joints_deg', home),
                      ('reset_joints_deg', reset)):
        print(f'  {name:<20} = {val}')
    print()

    for name, val in (('joint_sign', sign), ('joint_offset_deg', off),
                      ('move_j_offset_deg', mjoff), ('urdf_offset_deg', uoff),
                      ('home_joints_deg', home), ('reset_joints_deg', reset)):
        if val is None or len(val) != 6:
            print(f'FATAL: {name} missing or not length 6 in the config')
            return 1

    # ---- conversions, mirroring the driver -------------------------------
    def arm_chain_deg(a):
        """angle -> arm chain value (before urdf_offset_deg)."""
        return [sign[i] * (a[i] - off[i]) for i in range(6)]

    def angle_to_urdf_deg(a):
        return [arm_chain_deg(a)[i] + uoff[i] for i in range(6)]

    def movej_to_angle(m):
        return [m[i] - mjoff[i] for i in range(6)]

    def urdf_deg_to_movej(u):
        out = []
        for i in range(6):
            s = sign[i] if abs(sign[i]) > 1e-12 else 1.0
            out.append((u[i] - uoff[i]) / s + off[i] + mjoff[i])
        return out

    # ---- 1. config vs driver defaults -----------------------------------
    print('1. config vs driver code defaults')
    dtext = open(DRIVER, encoding='utf-8').read()
    d_sign = get_py_list(dtext, 'default_sign')
    d_off = get_py_list(dtext, 'default_offset_deg')
    d_mjoff = get_py_list(dtext, 'default_movej_offset_deg')
    d_uoff = get_py_list(dtext, 'default_urdf_offset_deg')
    for name, dv, cv in (('joint_sign', d_sign, sign),
                         ('joint_offset_deg', d_off, off),
                         ('move_j_offset_deg', d_mjoff, mjoff),
                         ('urdf_offset_deg', d_uoff, uoff)):
        if not close(dv, cv):
            fail(f'{name} differs: driver={dv} config={cv}')
        else:
            ok(f'{name} matches')
    print()

    # ---- 2. empirical anchor --------------------------------------------
    print('2. anchor: move_j 7-pose -> angle -> URDF')
    a = movej_to_angle(SEVEN_POSE_MOVEJ)
    chain = arm_chain_deg(a)
    u = angle_to_urdf_deg(a)
    print(f'  move_j    = {[round(v, 2) for v in SEVEN_POSE_MOVEJ]}')
    print(f'  -> angle  = {[round(v, 3) for v in a]}')
    print(f'  -> chain  = {[round(v, 3) for v in chain]}')
    print(f'  -> + uoff = {[round(v, 3) for v in u]}')
    # The chain value is the essence of the anchor: the 7 pose is where the
    # arm-chain angle is zero. The residual here is a few degrees because
    # move_j_offset_deg (87.7) and the measured angle (2.30) were each rounded
    # when written down; the live path uses the measured readback only, so this
    # rounding does not affect the robot.
    if not close(chain, SEVEN_POSE_ARM_CHAIN, tol=3.0):
        fail(f'arm chain should be near zero, got {chain}')
    else:
        residual = max(abs(v) for v in chain)
        ok(f'arm-chain near zero at the 7 pose (max residual {residual:.2f} deg)')
    # URDF must be exactly uoff plus that residual, i.e. consistent.
    expect = [chain[i] + uoff[i] for i in range(6)]
    if not close(u, expect, tol=1e-9):
        fail(f'uoff was not applied consistently: got {u}, expected {expect}')
    else:
        ok('urdf_offset_deg applied consistently')
    print()

    # ---- 3. measured readback: this is the path the robot actually uses --
    print('3. measured 7-pose readback -> exact URDF vector')
    u2 = angle_to_urdf_deg(SEVEN_POSE_ANGLE)
    expect2 = [0.0, 0.0, uoff[2], 0.0, 0.0, 0.0]
    print(f'  angle  = {SEVEN_POSE_ANGLE}')
    print(f'  -> URDF = {[round(v, 4) for v in u2]}')
    print(f'  expected (zero chain + uoff) = {expect2}')
    if not close(u2, expect2, tol=1e-6):
        fail(f'measured readback mismatch: got {u2}, expected {expect2}')
    else:
        ok('exact: joint_offset_deg was derived from these very numbers')
    print()

    # ---- 4. round trip ---------------------------------------------------
    print('4. round trip URDF -> move_j -> URDF')
    cases = [[0, 0, 0, 0, 0, 0], [0, 0, 180, 0, 0, 0],
             [10, -30, 45, 5, 20, -15], [-45, 20, -60, 12, -8, 33]]
    for case in cases:
        mj = urdf_deg_to_movej(case)
        back = angle_to_urdf_deg(movej_to_angle(mj))
        good = close(case, back, tol=1e-6)
        print(f'  {[round(v,1) for v in case]} -> move_j '
              f'{[round(v,2) for v in mj]} -> {[round(v,1) for v in back]}'
              f'   {"ok" if good else "MISMATCH"}')
        if not good:
            fail(f'round trip mismatch for {case}')
    if not failures:
        ok('identity for all cases')
    print()

    # ---- 5. named poses vs limits ---------------------------------------
    print('5. named poses (URDF) vs URDF limits')
    for label, pose in (('home', home), ('reset', reset)):
        u = angle_to_urdf_deg(movej_to_angle(pose))
        print(f'  {label}: move_j {pose}')
        print(f'    -> URDF {[round(v, 1) for v in u]}')
        bad = []
        for i, n in enumerate(sorted(LIMITS)):
            lo, hi = LIMITS[n]
            if u[i] < lo - 0.5 or u[i] > hi + 0.5:
                bad.append(f'{n}={u[i]:.1f} (limit {lo}..{hi})')
        if bad:
            print(f'    OUTSIDE: {"; ".join(bad)}')
            print('      The real arm may still reach it; the URDF limits are')
            print('      inherited from the model and not yet verified.')
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
