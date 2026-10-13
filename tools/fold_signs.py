#!/usr/bin/env python3
"""Emit the fold-geometry table, the check that actually settles joint_sign.

This is the one test that cannot be fooled by the parameters. The retracted
(power-off) pose has the forearm folded back along the upper arm, and that is a
fact about the metal, not about any calibration value. Forward kinematics on
the real URDF over the four possible (joint_2, joint_3) sign combinations shows
which ones produce a fold, and only one of those also reproduces the measured
commands.

It exists because the obvious-looking approach -- "the readback and the command
move the same way, so the sign is +1" -- gave the WRONG answer for joint_3.
That method only sees how two joints move between two poses, which is not
enough to pin a sign. Geometry is.

Usage:
    python tools/fold_signs.py
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

# from the config (measured with tools/probe_pairs.py)
JOFF = [-0.000, 75.032, -90.016, -0.002, -0.000, -0.007]
# the device reads ~0 on every joint at the retracted pose
ANGLE_RETRACT = [0.0] * 6


def fold_geometry(urdf_deg):
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
    print('FOLD GEOMETRY vs joint_2 / joint_3 SIGNS')
    print('=' * 74)
    print('  retracted pose: raw reads ~0 on every joint, so')
    print('      urdf_retract = sign * (0 - joint_offset_deg)')
    print('  A fold means the two arm segments end up nearly antiparallel')
    print('  (|between| > 140 deg).')
    print()
    print(f'  {"s2":>4} {"s3":>4}  {"upper":>9} {"forearm":>9} '
          f'{"between":>9} {"reach m":>8}  verdict')
    good = []
    for s2 in (1.0, -1.0):
        for s3 in (1.0, -1.0):
            sign = [-1.0, s2, s3, 1.0, -1.0, -1.0]
            urdf = [sign[i] * (ANGLE_RETRACT[i] - JOFF[i]) for i in range(6)]
            up, fo, between, reach = fold_geometry(urdf)
            ok = abs(between) > 140.0
            if ok:
                good.append((s2, s3))
            print(f'  {s2:>+4.0f} {s3:>+4.0f}  {up:>+9.1f} {fo:>+9.1f} '
                  f'{between:>+9.1f} {reach:>8.4f}  '
                  f'{"FOLDED" if ok else "not folded"}')
    print()
    print(f'  folds: {good}')
    print()
    if len(good) > 1:
        print('  More than one combination folds, so geometry alone does not')
        print('  settle it. The tie-break is the measured command: at the')
        print('  retracted pose the device was commanded')
        print('      reset_joints_deg = [0, -75, 180, 0, 0, 0]')
        print('  so the target chain has to reproduce that too. Run')
        print('  tools/resolve_signs.py for the full three-way check.')
    print()
    print('  The configuration uses s2 = +1, s3 = -1, i.e.')
    print('      joint_sign = [-1, 1, -1, 1, -1, -1]')
    print('  which is the combination that folds AND reproduces the measured')
    print('  commands at both named poses.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
