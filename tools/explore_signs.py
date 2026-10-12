#!/usr/bin/env python3
"""Rank joint_sign combinations by how well they reproduce the retracted pose.

Why this exists: the signs cannot be settled by algebra alone when the
"expected" pose is described in words. What CAN be done is to enumerate the
plausible sign combinations, push each through forward kinematics, and let a
human pick the one matching the real arm. This script does the enumeration and
prints the geometry so the choice is a matter of recognition, not reasoning.

Hard constraints applied first (these are not negotiable):

  C1 ANCHOR   At the 7 pose the model must read URDF all-zeros, which forces
              joint_offset_deg == angle_7pose. Reported as a check.

  C2 j2, j3   The retracted pose must be COMPACT: the tool tip near the base.
              At the retracted device readback (approximately all zeros) the
              URDF angles are sign * (-offset), so j2 and j3 signs decide
              whether the arm folds in or sticks out.

Usage:
    python tools/explore_signs.py
    python tools/explore_signs.py --fixed-j1 -1 --fixed-j6 -1
"""
import argparse
import contextlib
import io
import itertools
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
REPO = os.path.dirname(HERE)
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')

ANGLE_7POSE = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
OFFSET = list(ANGLE_7POSE)          # forced by C1
ANGLE_RETRACT = [0.0] * 6

with contextlib.redirect_stdout(io.StringIO()):
    import fk_dummy
    _LINKS, _JOINTS = fk_dummy.parse_model(XACRO)

_REV = sorted([j for j in _JOINTS if j['type'] in ('revolute', 'continuous')],
              key=lambda j: j['name'])
_NAMES = [j['name'] for j in _REV]


def geometry(urdf_deg):
    angles = {_NAMES[i]: math.radians(urdf_deg[i]) for i in range(6)}
    pts, T = fk_dummy.fk(_JOINTS, 'gripper_tcp', angles)
    sh, el, wr = pts['link2_3_v4_1'], pts['link4_v3_1'], pts['link7_v1_1']

    def ang(a, b):
        return math.degrees(math.atan2(b[2] - a[2], b[1] - a[1]))

    reach = math.hypot(T[0][3], T[1][3])
    return {
        'upper': ang(sh, el),
        'fore': ang(el, wr),
        'tcp': (T[0][3], T[1][3], T[2][3]),
        'reach': reach,
        'height': T[2][3],
    }


def word(d):
    if d > 60:
        return 'UP'
    if d > 25:
        return 'up-diag'
    if d > -25:
        return 'FWD'
    if d > -60:
        return 'down-diag'
    return 'DOWN'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fixed-j1', type=float, default=-1.0,
                    help='j1 sign, measured on hardware (default -1)')
    ap.add_argument('--fixed-j6', type=float, default=-1.0,
                    help='j6 sign, measured on hardware (default -1)')
    args = ap.parse_args()

    print('constraint C1: joint_offset_deg is forced to equal angle_7pose')
    print(f'  joint_offset_deg = {OFFSET}')
    urdf7 = [1.0 * (ANGLE_7POSE[i] - OFFSET[i]) for i in range(6)]
    print(f'  at the 7 pose URDF = {[round(v,4) for v in urdf7]}  '
          '(all zeros by construction)')
    print()

    print('constraint C2: the retracted pose must be COMPACT.')
    print('  retracted device readback is approximately all zeros, so')
    print('      urdf_retract[i] = sign[i] * (0 - offset[i])')
    print('  We enumerate sign choices for j2..j5 (j1 and j6 are pinned by')
    print('  hardware measurement) and rank by how compact the result is.')
    print()

    rows = []
    for s2, s3, s4, s5 in itertools.product([1.0, -1.0], repeat=4):
        sign = [args.fixed_j1, s2, s3, s4, s5, args.fixed_j6]
        urdf_r = [sign[i] * (ANGLE_RETRACT[i] - OFFSET[i]) for i in range(6)]
        # reject anything whose folded pose breaks the URDF limits badly
        g = geometry(urdf_r)
        rows.append((g['reach'], sign, urdf_r, g))

    rows.sort(key=lambda r: r[0])

    print(f'  {"rank":>4} {"signs":<28} {"reach":>7} {"height":>8}  '
          f'{"upper":>10} {"fore":>10}')
    for n, (reach, sign, urdf_r, g) in enumerate(rows[:8], 1):
        s = '[' + ','.join(f'{v:+.0f}' for v in sign) + ']'
        print(f'  {n:>4} {s:<28} {reach:>7.3f} {g["height"]:>+8.3f}  '
              f'{word(g["upper"]):>5} {g["upper"]:+5.1f} '
              f'{word(g["fore"]):>5} {g["fore"]:+5.1f}')

    print()
    print('  The retracted arm folds in toward the base, so the correct row is')
    print('  the one with the SMALLEST reach. Interpretation of the top row:')
    reach, sign, urdf_r, g = rows[0]
    s = '[' + ','.join(f'{v:+.0f}' for v in sign) + ']'
    print(f'    joint_sign = {s}')
    print(f'    retracted URDF = {[round(v,2) for v in urdf_r]}')
    print(f'    upper arm {word(g["upper"])} ({g["upper"]:+.1f} deg), '
          f'forearm {word(g["fore"])} ({g["fore"]:+.1f} deg)')
    print(f'    tip at {g["reach"]:.3f} m from the base axis, '
          f'{g["height"]:+.3f} m high')
    print()
    print('  Verify on the real arm: power off, look at the folded pose, and')
    print('  confirm the model shows the same shape. If the model folds the')
    print('  other way, drop to the next row.')
    print()
    print('  NOTE the URDF limits are exceeded by these poses (j3 reaches')
    print('  about +/-91 where the limit says 75). The limits are inherited')
    print('  from the model and are known to be narrower than the real arm.')
    print('  MoveIt will need them widened before it can plan these poses.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
