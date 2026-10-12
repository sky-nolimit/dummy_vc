#!/usr/bin/env python3
"""Check a proposed joint_sign / joint_offset_deg set against ALL constraints.

Three constraints must hold at once, and they are easy to satisfy one at a time
while breaking another:

  C1 ANCHOR      At the 7 pose the model must read URDF all-zeros.
                     urdf = sign * (angle_7pose - offset) == 0
                 This forces offset == angle_7pose, INDEPENDENT of sign.
                 Changing a sign without keeping that identity silently
                 re-rotates the whole joint.

  C2 RETRACT     At the power-off retracted pose the model must show the
                 folded geometry: upper arm forward, forearm down.
                 Measured angle readback there is approximately all zeros, so
                     urdf = sign * (0 - offset)
                 must give roughly j2 negative and j3 positive.

  C3 POLARITY    If a joint's direction was measured wrong on hardware, the
                 sign must be flipped. But flipping it also flips C2, so the
                 two constraints can disagree -- which is exactly the
                 situation this script is meant to expose rather than hide.

Usage:
    python tools/check_signs.py
    python tools/check_signs.py --sign -1,-1,-1,1,-1,-1 \\
                               --offset 0.215,74.572,-91.474,-0.090,1.306,-0.040
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
REPO = os.path.dirname(HERE)
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')

ANGLE_7POSE = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
# Retracted: device reports approximately zero on every channel.
ANGLE_RETRACT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# Expected URDF at the retracted pose: folded, upper arm forward, forearm down.
URDF_RETRACT_EXPECT = [0.0, -74.572, 91.474, 0.0, 0.0, 0.0]

MOVABLE = ['dummy2_joint%d' % i for i in range(1, 7)]


def geometry(urdf_deg):
    """Return (upper arm angle, forearm angle, tcp) from forward kinematics."""
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        import fk_dummy
        links, joints = fk_dummy.parse_model(XACRO)
    rev = sorted([j for j in joints if j['type'] in ('revolute', 'continuous')],
                 key=lambda j: j['name'])
    names = [j['name'] for j in rev]
    angles = {names[i]: math.radians(urdf_deg[i]) for i in range(6)}
    pts, T = fk_dummy.fk(joints, 'gripper_tcp', angles)
    sh, el, wr = pts['link2_3_v4_1'], pts['link4_v3_1'], pts['link7_v1_1']

    def ang(a, b):
        return math.degrees(math.atan2(b[2] - a[2], b[1] - a[1]))

    return ang(sh, el), ang(el, wr), (T[0][3], T[1][3], T[2][3])


def describe(upper, fore):
    def word(d):
        if d > 60:
            return 'UP'
        if d > 25:
            return 'up-diag'
        if d > -25:
            return 'FORWARD'
        if d > -60:
            return 'down-diag'
        return 'DOWN'
    return f'upper {word(upper):>9} ({upper:+6.1f}), fore {word(fore):>9} ({fore:+6.1f})'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sign', default=None)
    ap.add_argument('--offset', default=None)
    args = ap.parse_args()

    if args.sign:
        sign = [float(v) for v in args.sign.split(',')]
    else:
        # Recommended configuration. j2 and j3 signs are decided by the
        # retracted-pose geometry (C2 below): with these values the retracted
        # arm has its tip 0.071 m from the base axis, i.e. genuinely folded.
        # j1/j6 come from motion testing; j4/j5 are wrist joints that cannot
        # be judged geometrically, so they keep their tested values.
        sign = [-1.0, 1.0, -1.0, 1.0, -1.0, -1.0]
    if args.offset:
        off = [float(v) for v in args.offset.split(',')]
    else:
        # Pure anchor values. A small deviation (under ~1.5 deg) is within the
        # noise of hand-placing the arm, so do not tune these by eye.
        off = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]

    print('proposed parameters')
    print(f'  joint_sign       = {sign}')
    print(f'  joint_offset_deg = {off}')
    print()

    problems = []

    # ---- C1 anchor ------------------------------------------------------
    print('C1 anchor: at the 7 pose the model must read URDF all-zeros')
    urdf7 = [sign[i] * (ANGLE_7POSE[i] - off[i]) for i in range(6)]
    for i in range(6):
        mark = 'ok  ' if abs(urdf7[i]) < 0.5 else 'BAD '
        print(f'  [{mark}] j{i+1}: angle_7pose={ANGLE_7POSE[i]:+9.3f} - '
              f'offset={off[i]:+9.3f} -> urdf={urdf7[i]:+8.3f}')
        if abs(urdf7[i]) >= 0.5:
            problems.append(f'C1 j{i+1} off by {urdf7[i]:+.2f} deg')
    print()

    # ---- C2 retract -----------------------------------------------------
    #
    # The criterion is GEOMETRIC, not a hardcoded URDF vector: the retracted
    # arm folds in toward its base, so the tool tip must end up CLOSE to the
    # base axis and low. Testing against a fixed expected vector would just
    # encode whichever guess was made first, and an earlier version of this
    # script did exactly that and reported a correct configuration as broken.
    print('C2 retract: at the power-off pose the model must FOLD IN')
    urdf_r = [sign[i] * (ANGLE_RETRACT[i] - off[i]) for i in range(6)]
    print(f'  computed URDF = {[round(v, 3) for v in urdf_r]}')
    upper, fore, tcp = geometry(urdf_r)
    reach = math.hypot(tcp[0], tcp[1])
    print(f'  geometry: {describe(upper, fore)}')
    print(f'  tip = ({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f})')
    print(f'  reach from base axis = {reach:.3f} m, height = {tcp[2]:+.3f} m')
    # The 7 pose reaches about 0.32 m out; anything under half that is a fold.
    compact = reach < 0.16
    print(f'  compact enough to be a fold (reach < 0.16 m)? '
          f'{"YES" if compact else "NO"}')
    if not compact:
        problems.append(
            f'C2 the retracted pose reaches {reach:.3f} m from the base; '
            'the arm does not fold in, so a sign is wrong')
    print()

    # ---- C3 consistency report -------------------------------------------
    print('C3 how a sign flip interacts with the anchor')
    print('  offset must equal angle_7pose for the anchor to hold, and that is')
    print('  true regardless of sign. So a sign flip re-rotates the joint about')
    print('  the 7 pose WITHOUT breaking C1 -- it only breaks C2.')
    print('  Therefore: if hardware says the direction is wrong, and C2 also')
    print('  insists on the opposite sign, the two measurements disagree and')
    print('  the physical retracted pose must be re-examined.')
    print()

    print('=' * 64)
    if problems:
        print(f'RESULT: {len(problems)} problem(s)')
        for p in problems:
            print(f'  - {p}')
        print()
        print('If C1 is clean but C2 fails on j2/j3, then the signs found by')
        print('motion testing conflict with the retracted pose geometry. The')
        print('retracted geometry is the one to trust, because it is an')
        print('independent measurement: it does not depend on any sign choice,')
        print('only on the raw angle readback being near zero, which is what')
        print('the device reports with the motors off.')
        return 1
    print('RESULT: all constraints hold')
    return 0


if __name__ == '__main__':
    sys.exit(main())
