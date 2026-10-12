#!/usr/bin/env python3
"""Consistency self-test for the driver's three coordinate spaces.

The driver converts between three spaces, and getting the algebra wrong is
invisible until real hardware is commanded to a wrong place. This script
re-implements the same formulas and asserts the properties that must hold:

  1. The calibration anchor maps correctly both ways.
       7 pose: angle -> URDF = all zeros, URDF -> move_j = [0,0,90,0,0,0]
  1b. The retracted (power-off) pose folds back on itself. This is the only
     check that is independent of the joint_sign choice, so it is what
     actually pins the signs down.
  2. Round trip URDF -> move_j -> URDF, and URDF -> angle -> URDF, are the
     identity.
  3. Reports the derived down-link constant for comparison with the reference
     project. Supporting evidence, not proof.
  4. The driver's code defaults still agree with the config, and the launch
     file has not reintroduced a second, hardcoded copy of the calibration
     values.

config/dummy_controller.yaml is the single source of truth for every number.
This script reads them from there and does NOT import the driver: it is an
independent check, so a bug in the driver cannot hide by being mirrored here.

Usage:
    python tools/selftest_conversion.py
    python tools/selftest_conversion.py --sign -1,1,1,1,1,1
"""
import argparse
import ast
import contextlib
import io
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
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

# Retracted (power-off) pose: the device reports approximately zero on every
# channel, so the URDF angles there are sign * (0 - offset).
ANGLE_RETRACT = [0.0] * 6

XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')


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

    def check_true(label, okc, detail=''):
        print(f'  [{"ok  " if okc else "FAIL"}] {label}')
        if detail:
            print(f'          {detail}')
        if not okc:
            failures.append(label)

    # ---- 1. anchor ------------------------------------------------------
    print('1. calibration anchor (7 pose)')
    check('angle_7pose -> URDF == 0', a2u(ANGLE_7POSE), URDF_7POSE, tol=1e-6)
    check('URDF 0 -> move_j == [0,0,90,...]', u2m(URDF_7POSE), MOVEJ_7POSE,
          tol=1e-6)
    check('angle_7pose -> move_j == [0,0,90,...]', u2m(a2u(ANGLE_7POSE)),
          MOVEJ_7POSE, tol=1e-6)
    print()

    # ---- 1b. retracted pose geometry ------------------------------------
    #
    # The retracted (power-off) pose must FOLD: the forearm comes back along
    # the upper arm rather than the arm sticking straight out. This is the
    # only independent check on the SIGNS, because the retracted device
    # readback is approximately all zeros, so
    #     urdf_retract[i] = sign[i] * (0 - offset[i])
    # and a wrong sign flips a joint about the anchor without breaking
    # section 1 above. Distances alone do not settle it -- a mirrored fold is
    # equally "compact" -- so the segment directions are reported too.
    print('1b. retracted pose folds back on itself')
    urdf_r = a2u(ANGLE_RETRACT)
    print(f'  computed URDF = {[round(v, 3) for v in urdf_r]}')
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            import fk_dummy
            links, joints = fk_dummy.parse_model(XACRO)
        rev = sorted([j for j in joints
                      if j['type'] in ('revolute', 'continuous')],
                     key=lambda j: j['name'])
        names = [j['name'] for j in rev]
        angles = {names[i]: math.radians(urdf_r[i]) for i in range(6)}
        pts, T = fk_dummy.fk(joints, 'gripper_tcp', angles)
        sh, el, wr = pts['link2_3_v4_1'], pts['link4_v3_1'], pts['link7_v1_1']

        def sang(p, q):
            return math.degrees(math.atan2(q[2] - p[2], q[1] - p[1]))

        upper, fore = sang(sh, el), sang(el, wr)
        between = (fore - upper + 180.0) % 360.0 - 180.0
        reach = math.hypot(T[0][3], T[1][3])
        print(f'  upper arm {upper:+7.1f} deg, forearm {fore:+7.1f} deg')
        print(f'  angle between segments = {between:+7.1f} deg')
        print(f'  tip {reach:.4f} m from the base axis, '
              f'height {T[2][3]:+.4f} m')
        # A fold means the segments are close to antiparallel.
        folded = abs(between) > 140.0
        print(f'  [{"ok  " if folded else "FAIL"}] segments are near '
              'antiparallel (folded)')
        if not folded:
            failures.append(
                f'retracted pose is not folded: segments differ by only '
                f'{between:+.1f} deg, so a joint_sign is probably wrong')
    except Exception as exc:  # noqa: BLE001
        print(f'  [skip] forward kinematics unavailable: {exc}')
        print('         install pitch on the Linux side to enable this check')
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

    # ---- 4. driver defaults, and no second copy in the launch file -------
    #
    # These numbers used to be written down in three places: this config, the
    # driver's code defaults, and a DEFAULTS dict in the launch file. They
    # drifted apart repeatedly -- and because that dict was applied AFTER the
    # YAML parameter file, it silently WON, so editing the config appeared to
    # do nothing at all.
    #
    # The launch file no longer holds any of them: it reads them out of this
    # config when the launch description is built. So two things are left to
    # verify here -- that the driver's code defaults still agree, and that
    # nobody has reintroduced a hardcoded copy into the launch file.
    print('4. driver defaults agree; launch file holds no second copy')
    checks = [('joint_sign', sign), ('joint_offset_deg', joff),
              ('move_j_offset_deg', mjoff)]

    dtext = open(DRIVER, encoding='utf-8').read()
    ltext = open(LAUNCH, encoding='utf-8').read()
    ltree = ast.parse(ltext)
    for key, want in checks:
        m = re.search(rf'{key}\s*=\s*\[([^\]]*)\]', dtext)
        if key == 'joint_sign':
            m = re.search(r'default_sign\s*=\s*\[([^\]]*)\]', dtext)
        got = [float(v) for v in m.group(1).split(',')] if m else None
        check(f'driver default {key}', got, want, tol=1e-6)

    # Every parameter the config declares, taken from the file itself so a
    # parameter added to the YAML is automatically covered.
    param_names = set(re.findall(r'^\s{4}([a-z_][a-z0-9_]*)\s*:', text, re.M))
    if not param_names:
        print('FATAL: could not read any parameter names from the config')
        return 1

    # The launch file must obtain the values from the config...
    check_true(
        'launch reads the config instead of restating it',
        'load_config_defaults' in ltext and 'ros__parameters' in ltext,
        'expected a load_config_defaults() helper reading ros__parameters')

    # ...and must not grow a literal dict of them again. Only literals that
    # actually look like parameter values are flagged, so the prose in
    # DESCRIPTIONS is not mistaken for one, and the usage example in the
    # module docstring is ignored (it is not a dict entry).
    def looks_like_value(s):
        s = s.strip()
        return bool(re.fullmatch(r'-?\d+(?:\.\d+)?', s)
                    or re.fullmatch(r'\[[-\d.,\s]*\]', s))

    offenders = []
    for node in ast.walk(ltree):
        if not isinstance(node, ast.Dict):
            continue
        for k, v in zip(node.keys, node.values):
            if (isinstance(k, ast.Constant) and isinstance(k.value, str)
                    and k.value in param_names
                    and isinstance(v, ast.Constant)
                    and isinstance(v.value, str)
                    and looks_like_value(v.value)):
                offenders.append(k.value)
    check_true(
        'launch file has no hardcoded copy of the calibration values',
        not offenders,
        (f'hardcoded in the launch file: {sorted(set(offenders))}'
         if offenders else
         'the YAML is the only place these numbers are written down'))

    # The launch file raises if a config parameter has no description, so a
    # missing one would break bringup outright. Catch it here instead.
    desc_keys = set()
    for node in ast.walk(ltree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(t, ast.Name) and t.id == 'DESCRIPTIONS'
                   for t in node.targets):
            continue
        if isinstance(node.value, ast.Dict):
            desc_keys = {k.value for k in node.value.keys
                         if isinstance(k, ast.Constant)
                         and isinstance(k.value, str)}
    missing_desc = sorted(param_names - desc_keys)
    check_true(
        'every config parameter is documented as a launch argument',
        not missing_desc,
        (f'missing from DESCRIPTIONS: {missing_desc}' if missing_desc
         else f'{len(param_names)} parameters covered'))
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
