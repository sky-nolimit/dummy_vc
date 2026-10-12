#!/usr/bin/env python3
"""Identify the URDF pose that matches a described arm shape.

Why this exists: "compact" turned out to be an insufficient criterion. The real
retracted pose folds the forearm BACK along the upper arm -- the two segments
are roughly antiparallel -- rather than merely bringing the tip near the base.
A tip-distance check cannot tell those apart, and an earlier analysis picked
the wrong one because of it.

This script reports, for a given URDF pose, the two segment directions and the
angle BETWEEN the segments, which is what distinguishes a fold:

    angle between segments near 0    -> arm extended straight
    angle between segments near +/-90 -> right angle (the 7 pose)
    angle between segments near +/-180 -> folded back on itself (retracted)

Usage:
    python tools/find_fold.py                        # scan and rank by fold
    python tools/find_fold.py --deg 0,-74.57,-91.47,0,0,0
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
ANGLE_RETRACT = [0.0] * 6

with contextlib.redirect_stdout(io.StringIO()):
    import fk_dummy
    _LINKS, _JOINTS = fk_dummy.parse_model(XACRO)

_REV = sorted([j for j in _JOINTS if j['type'] in ('revolute', 'continuous')],
              key=lambda j: j['name'])
_NAMES = [j['name'] for j in _REV]


def analyse(urdf_deg):
    angles = {_NAMES[i]: math.radians(urdf_deg[i]) for i in range(6)}
    pts, T = fk_dummy.fk(_JOINTS, 'gripper_tcp', angles)
    sh, el, wr = pts['link2_3_v4_1'], pts['link4_v3_1'], pts['link7_v1_1']

    def ang(a, b):
        return math.degrees(math.atan2(b[2] - a[2], b[1] - a[1]))

    upper = ang(sh, el)
    fore = ang(el, wr)
    # signed angle between the segments, wrapped to (-180, 180]
    between = (fore - upper + 180.0) % 360.0 - 180.0
    return {
        'upper': upper,
        'fore': fore,
        'between': between,
        'reach': math.hypot(T[0][3], T[1][3]),
        'height': T[2][3],
        'tcp': (T[0][3], T[1][3], T[2][3]),
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
    ap.add_argument('--deg', default=None)
    args = ap.parse_args()

    if args.deg:
        deg = [float(v) for v in args.deg.split(',')]
        a = analyse(deg)
        print(f'URDF deg = {deg}')
        print(f'  upper   {word(a["upper"]):>9} ({a["upper"]:+7.1f})')
        print(f'  fore    {word(a["fore"]):>9} ({a["fore"]:+7.1f})')
        print(f'  angle between segments = {a["between"]:+7.1f} deg')
        print(f'  tip reach {a["reach"]:.3f} m, height {a["height"]:+.3f} m')
        return 0

    print('The retracted pose in the photo folds the forearm BACK along the')
    print('upper arm: the two segments are roughly antiparallel, i.e. the')
    print('angle between them is near +/-180. Scanning sign choices to find')
    print('which one produces that, using the retracted device readback')
    print('(approximately all zeros) as the input angles.')
    print()
    print('  urdf_retract[i] = sign[i] * (0 - offset[i]),  offset = angle_7pose')
    print()

    off = ANGLE_7POSE
    rows = []
    for combo in itertools.product([1.0, -1.0], repeat=6):
        urdf_r = [combo[i] * (ANGLE_RETRACT[i] - off[i]) for i in range(6)]
        a = analyse(urdf_r)
        rows.append((abs(abs(a['between']) - 180.0), combo, urdf_r, a))

    rows.sort(key=lambda r: r[0])

    print(f'  {"signs":<26} {"betw":>8} {"upper":>6} {"fore":>6} '
          f'{"reach":>7} {"height":>8}')
    seen = set()
    shown = 0
    for score, sign, urdf_r, a in rows:
        key = (round(a['between'], 1), round(a['upper'], 1))
        if key in seen:
            continue
        seen.add(key)
        s = '[' + ','.join(f'{v:+.0f}' for v in sign) + ']'
        print(f'  {s:<26} {a["between"]:>+8.1f} {a["upper"]:>+6.1f} '
              f'{a["fore"]:>+6.1f} {a["reach"]:>7.3f} {a["height"]:>+8.3f}')
        shown += 1
        if shown >= 10:
            break

    print()
    print('Interpretation: the photo shows the upper arm roughly horizontal')
    print('and the forearm folded back over it, so look for a row with')
    print('"between" near +/-180 and "upper" near 0 or +/-180.')
    print()
    print('NOTE the URDF limits are still violated by these poses (j3 reaches')
    print('about +/-91 where the model limit says 75). The limits are inherited')
    print('and are known to be narrower than the real arm.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
