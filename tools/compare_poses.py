#!/usr/bin/env python3
"""Compare candidate URDF poses against the two measured device readbacks.

The situation
-------------
Two device readbacks were measured (angle space, degrees):

    retracted   [-0.0004,  0.1564, -0.1264,  0.0013, 0.0015, -0.0030]
    7 pose      [ 0.2146, 74.5723, -91.4737, -0.9101, -0.0347, -0.0401]

A photo confirms the 7 pose is: upper arm VERTICAL, forearm HORIZONTAL.
Forward kinematics on this URDF says the zero pose [0,0,0,0,0,0] is exactly
"upper arm vertical, forearm horizontal". So the 7 pose should be URDF all
zeros, and the mapping device -> URDF is a pure offset:

    urdf_deg = device_deg - offset_deg,   offset = the 7-pose readback

which gives, for the retracted readback:

    urdf_deg = [0, 0, 0, ...] - [0.21, 74.57, -91.47, ...]
             = [-0.21, -74.57, +91.47, +0.91, +0.03, +0.04]

This script checks whether that predicted retracted pose is geometrically
sane (upper arm down/forward, forearm tucked) and reports the segment angles,
so the assumption can be confirmed or refuted instead of guessed at.

Efficiency note: the URDF is parsed ONCE. An earlier version re-parsed it in
the inner loop, which both was slow and buried the output in xacro warnings.

Usage:
    python tools/compare_poses.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
REPO = os.path.dirname(HERE)
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')

# Device readbacks, angle space, degrees.
RETRACTED = [-0.0004, 0.1564, -0.1264, 0.0013, 0.0015, -0.0030]
SEVEN = [0.2146, 74.5723, -91.4737, -0.9101, -0.0347, -0.0401]

SHOULDER = 'link2_3_v4_1'
ELBOW = 'link4_v3_1'
WRIST = 'link7_v1_1'

# Import once, and silence nothing: fk_dummy.parse_model is called a single
# time below, so its unresolved-xacro warning appears once, not per iteration.
import contextlib
import io

with contextlib.redirect_stdout(io.StringIO()):
    import fk_dummy
    LINKS, JOINTS = fk_dummy.parse_model(XACRO)

REV = sorted([j for j in JOINTS if j['type'] in ('revolute', 'continuous')],
             key=lambda j: j['name'])
NAMES = [j['name'] for j in REV]


def geometry(urdf_deg):
    """Segment angles and TCP for a URDF pose given in degrees."""
    angles = {NAMES[i]: math.radians(urdf_deg[i]) for i in range(6)}
    pts, T = fk_dummy.fk(JOINTS, 'gripper_tcp', angles)
    sh, el, wr = pts[SHOULDER], pts[ELBOW], pts[WRIST]

    def ang(a, b):
        return math.degrees(math.atan2(b[2] - a[2], b[1] - a[1]))

    return {
        'upper': ang(sh, el),
        'fore': ang(el, wr),
        'upper_len': math.dist(sh, el),
        'fore_len': math.dist(el, wr),
        'tcp': (T[0][3], T[1][3], T[2][3]),
    }


def word(deg):
    if deg > 60:
        return 'UP'
    if deg > 25:
        return 'up-diagonal'
    if deg > -25:
        return 'FORWARD'
    if deg > -60:
        return 'down-diagonal'
    return 'DOWN'


def report(label, urdf_deg):
    g = geometry(urdf_deg)
    print(f'  {label}')
    print(f'    URDF deg     = {[round(v, 2) for v in urdf_deg]}')
    print(f'    upper arm    = {word(g["upper"]):>13} ({g["upper"]:+7.1f} deg)'
          f'  len {g["upper_len"]:.3f} m')
    print(f'    forearm      = {word(g["fore"]):>13} ({g["fore"]:+.1f} deg)'
          f'  len {g["fore_len"]:.3f} m')
    print(f'    TCP          = ({g["tcp"][0]:+.3f}, {g["tcp"][1]:+.3f}, '
          f'{g["tcp"][2]:+.3f})')
    return g


def main():
    print(f'model: {os.path.relpath(XACRO, REPO)}')
    print()
    print('Segment angles are measured in the sagittal (y,z) plane:')
    print('   0 deg = pointing FORWARD (+y),  +90 deg = pointing UP (+z)')
    print()

    print('measured device readbacks (angle space, degrees)')
    print(f'  retracted = {RETRACTED}')
    print(f'  7 pose    = {SEVEN}')
    print()

    # ---- step 1: confirm the zero pose is the 7 pose ---------------------
    print('1. is URDF zero really the geometry in the photo?')
    g_zero = report('[0,0,0,0,0,0]', [0.0] * 6)
    ok_shape = g_zero['upper'] > 60 and abs(g_zero['fore']) < 25
    print(f'    -> "upper arm UP + forearm FORWARD" ? '
          f'{"YES" if ok_shape else "NO"}')
    print()

    # ---- step 2: the offset implied by anchoring at the 7 pose -----------
    offset = SEVEN
    print('2. anchoring URDF zero at the 7 pose gives offset = 7-pose readback')
    print(f'    offset_deg = {[round(v, 3) for v in offset]}')
    print()

    print('3. predicted poses under that offset')
    u_seven = [SEVEN[i] - offset[i] for i in range(6)]
    u_ret = [RETRACTED[i] - offset[i] for i in range(6)]
    report('7 pose (should equal URDF zero)', u_seven)
    print()
    g_ret = report('retracted (prediction)', u_ret)
    print()

    looks_retracted = (g_ret['upper'] < 25) and (g_ret['fore'] < -25)
    print(f'    -> does it look retracted (upper arm not up, '
          f'forearm angled down)?  {"YES" if looks_retracted else "NO"}')
    print()

    # ---- step 4: also show what the device would need for a folded pose --
    print('4. for reference, URDF poses that LOOK retracted on this model')
    cands = [(0, -75, -75), (0, -90, -75), (0, -75, -90), (0, -90, 0),
             (0, -75, 0)]
    for c in cands:
        deg = list(c) + [0, 0, 0]
        g = geometry(deg)
        tag = ''
        if g['upper'] < 25 and g['fore'] < -25:
            tag = '  <-- plausible retracted'
        print(f'    j2={c[1]:+4d} j3={c[2]:+4d}  upper {word(g["upper"]):>13}'
              f' ({g["upper"]:+6.1f})  fore {word(g["fore"]):>13}'
              f' ({g["fore"]:+6.1f})  TCP=({g["tcp"][0]:+.3f},'
              f'{g["tcp"][1]:+.3f},{g["tcp"][2]:+.3f}){tag}')
    print()

    print('=' * 66)
    print('INTERPRETATION')
    print('  If step 1 is YES and step 3 says the prediction looks retracted,')
    print('  the offset anchor is consistent and can be used directly:')
    print(f'    joint_offset_deg = {[round(v, 3) for v in offset]}')
    print('    joint_sign       = all +1')
    print('  If the prediction does NOT look retracted, the two readbacks are')
    print('  not related by a pure offset (gain is not 1), and at least one')
    print('  more measured pose is needed to pin the gain down.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
