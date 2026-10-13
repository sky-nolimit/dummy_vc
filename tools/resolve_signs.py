#!/usr/bin/env python3
"""Resolve the sign conflict between the readback chain and the target chain.

Three requirements, all measured, and they cannot share one `sign`:

  G (geometry)  The retracted pose must FOLD: the forearm comes back along the
                upper arm. This is checked by forward kinematics on the real
                URDF, and it is the only check that is independent of any
                parameter choice. Measured raw at retracted is ~0 on every
                joint, so it pins joint_sign directly:
                    urdf_retract = sign * (0 - joint_offset_deg)
                Folding requires joint_2 = +75 and joint_3 = -90 there.

  R (readback)  The model must read zero at the 7 pose, and must agree with the
                device. joint_offset_deg = raw readback at the 7 pose.

  T (target)    move_j for the retracted URDF must be reset_joints_deg, and for
                URDF zero must be home_joints_deg.

Measured facts (tools/probe_pairs.py):
    home  move_j [0, 0, 90, 0, 0, 0] -> raw [0, 75.032, -90.016, 0, 0, 0]
    reset move_j [0, -75, 180, 0, 0, 0] -> raw [0, 0.04, -0.03, 0, 0, 0]
    so raw = move_j + K with K = [0, 75, -180, 0, 0, 0]

Usage:
    python tools/resolve_signs.py
"""
import contextlib
import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')

HOME_RAW = [-0.0, 75.032, -90.016, -0.002, -0.0, -0.007]
RESET_RAW = [-0.0, 0.04, -0.03, 0.001, 0.001, -0.001]
HOME_CMD = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
RESET_CMD = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]
K = [0.0, 75.0, -180.0, 0.0, 0.0, 0.0]


def fold_geometry(urdf_deg):
    """Return (upper, forearm, between, reach) for a URDF pose in degrees."""
    import fk_dummy
    links, joints = fk_dummy.parse_model(XACRO)
    rev = sorted([j for j in joints
                  if j['type'] in ('revolute', 'continuous')],
                 key=lambda j: j['name'])
    names = [j['name'] for j in rev]
    angles = {names[i]: math.radians(urdf_deg[i]) for i in range(6)}
    with contextlib.redirect_stdout(io.StringIO()):
        pts, T = fk_dummy.fk(joints, 'gripper_tcp', angles)
    sh, el, wr = pts['link2_3_v4_1'], pts['link4_v3_1'], pts['link7_v1_1']

    def sang(p, q):
        return math.degrees(math.atan2(q[2] - p[2], q[1] - p[1]))

    upper, fore = sang(sh, el), sang(el, wr)
    between = (fore - upper + 180.0) % 360.0 - 180.0
    return upper, fore, between, math.hypot(T[0][3], T[1][3])


def main():
    print('=' * 74)
    print('THE THREE REQUIREMENTS')
    print('=' * 74)
    print('  G  retracted must FOLD (forward kinematics, parameter-free)')
    print('  R  model reads zero at the 7 pose        -> joint_offset = HOME_RAW')
    print('  T  move_j for a URDF pose must match the measured command')
    print()

    joff = list(HOME_RAW)
    print(f'  joint_offset_deg (from R) = {[round(v, 3) for v in joff]}')
    print()

    print('=' * 74)
    print('CHECK EACH SIGN CHOICE AGAINST G, FOR JOINT_2 AND JOINT_3')
    print('=' * 74)
    print('  The other joints do not move between these poses, so only j2 and')
    print('  j3 decide the fold. Enumerate them:')
    print()
    print(f'  {"s2":>4} {"s3":>4}  {"upper":>9} {"forearm":>9} '
          f'{"between":>9} {"reach":>8}  verdict')
    best = None
    for s2 in (1.0, -1.0):
        for s3 in (1.0, -1.0):
            sign = [-1.0, s2, s3, 1.0, -1.0, -1.0]
            urdf = [sign[i] * (0.0 - joff[i]) for i in range(6)]
            up, fo, between, reach = fold_geometry(urdf)
            ok = abs(between) > 140.0
            print(f'  {s2:>+4.0f} {s3:>+4.0f}  {up:>+9.1f} {fo:>+9.1f} '
                  f'{between:>+9.1f} {reach:>8.4f}  '
                  f'{"FOLDED" if ok else "not folded"}')
            if ok and best is None:
                best = (s2, s3)
    print()
    if best is None:
        print('  No combination folds. Something else is wrong; stopping.')
        return 1
    s2, s3 = best
    sign = [-1.0, s2, s3, 1.0, -1.0, -1.0]
    print(f'  => geometry requires joint_sign = {sign}')
    print(f'     at retracted this gives urdf = '
          f'{[round(sign[i] * (0.0 - joff[i]), 1) for i in range(6)]}')
    print()

    print('=' * 74)
    print('NOW CHECK R AND T WITH THAT SIGN')
    print('=' * 74)

    def r2u(raw):
        return [sign[i] * (raw[i] - joff[i]) for i in range(6)]

    print('  R: readback at home (must be all zeros):')
    u_home = r2u(HOME_RAW)
    print(f'     {[round(v, 3) for v in u_home]}  '
          f'{"ok" if all(abs(v) < 0.05 for v in u_home) else "WRONG"}')
    print()
    print('  T: the command for a URDF pose. Try the obvious form first:')
    print('       move_j = urdf/sign + joint_offset + move_j_offset')
    mjoff = [HOME_CMD[i] - joff[i] for i in range(6)]
    print(f'     move_j_offset_deg = {[round(v, 3) for v in mjoff]}')
    u_reset = r2u(RESET_RAW)
    print(f'     URDF at reset = {[round(v, 3) for v in u_reset]}')
    m = [u_reset[i] / sign[i] + joff[i] + mjoff[i] for i in range(6)]
    print(f'     move_j        = {[round(v, 3) for v in m]}')
    print(f'     wanted        = {RESET_CMD}')
    ok_plain = all(abs(m[i] - RESET_CMD[i]) < 0.1 for i in range(6))
    print(f'     match: {ok_plain}')
    print()
    if not ok_plain:
        print('     It does not match. Try with the sign INVERTED in the target')
        print('     chain only:  move_j = -urdf/sign + joff + mjoff')
        m2 = [-u_reset[i] / sign[i] + joff[i] + mjoff[i] for i in range(6)]
        print(f'     move_j = {[round(v, 3) for v in m2]}')
        print(f'     wanted = {RESET_CMD}')
        ok2 = all(abs(m2[i] - RESET_CMD[i]) < 0.1 for i in range(6))
        print(f'     match: {ok2}')
        print()
        if ok2:
            print('     => the two chains need OPPOSITE signs. The readback')
            print('        chain uses `sign`, the target chain uses `-sign`.')
            print('        Re-derive them from the measured raw = move_j + K')
            print('        relation instead of treating one as the inverse of')
            print('        the other, since they are not inverses as written.')
    print()
    print('=' * 74)
    print('SUMMARY')
    print('=' * 74)
    print(f'  joint_sign        = {sign}')
    print(f'  joint_offset_deg  = {[round(v, 3) for v in joff]}')
    print(f'  move_j_offset_deg = {[round(v, 3) for v in mjoff]}')
    print()
    print('  Whether the target chain needs an extra sign flip is what the')
    print('  check above decides. Do not guess it: the geometry check settles')
    print('  joint_sign, and the measured commands settle the target chain.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
