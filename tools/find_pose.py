#!/usr/bin/env python3
"""Search for the URDF joint angles that produce a described arm shape.

Why this exists: several rounds were lost guessing which URDF pose matched
"the 7 pose". Describing a pose in words is ambiguous; describing it as
"upper arm vertical, forearm horizontal" is a geometric constraint that can be
solved for directly against the real URDF.

The arm chain in this model has two meaningful segments:
    shoulder -> elbow   (link2_3_v4_1 -> link4_v3_1)
    elbow    -> wrist   (link5_v3_1    -> link7_v1_1)

This script sweeps j2 and j3 (the two pitch joints) over their range and
reports every configuration whose segment directions match a target shape.

Usage:
    python tools/find_pose.py                     # list candidate shapes
    python tools/find_pose.py --shape seven       # upper arm up, forearm out
    python tools/find_pose.py --shape retracted   # upper arm down/forward
    python tools/find_pose.py --shape zero        # print the zero pose geometry
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

SHOULDER = 'link2_3_v4_1'
ELBOW = 'link4_v3_1'
WRIST = 'link7_v1_1'


def analyse(deg_j2, deg_j3):
    """Return segment direction descriptions for given j2/j3 (degrees)."""
    import fk_dummy

    links, joints = fk_dummy.parse_model(XACRO)
    rev = sorted([j for j in joints if j['type'] in ('revolute', 'continuous')],
                 key=lambda j: j['name'])
    names = [j['name'] for j in rev]
    angles = {n: 0.0 for n in names}
    angles[names[1]] = math.radians(deg_j2)   # dummy2_joint2
    angles[names[2]] = math.radians(deg_j3)   # dummy2_joint3

    pts, T = fk_dummy.fk(joints, 'gripper_tcp', angles)
    if not all(k in pts for k in (SHOULDER, ELBOW, WRIST)):
        return None

    sh, el, wr = pts[SHOULDER], pts[ELBOW], pts[WRIST]
    # project onto the sagittal plane (y = forward, z = up); j1 is 0 here
    upper = (el[1] - sh[1], el[2] - sh[2])
    fore = (wr[1] - el[1], wr[2] - el[2])

    def deg(v):
        return math.degrees(math.atan2(v[1], v[0]))

    return {
        'upper_ang': deg(upper),          # 0 = forward, 90 = up
        'fore_ang': deg(fore),
        'upper_len': math.hypot(*upper),
        'fore_len': math.hypot(*fore),
        'tcp': (T[0][3], T[1][3], T[2][3]),
    }


def describe(a):
    ua, fa = a['upper_ang'], a['fore_ang']

    def word(ang):
        if ang > 60:
            return 'UP'
        if ang < -60:
            return 'DOWN'
        if abs(ang) < 30:
            return 'FORWARD'
        return 'diagonal'

    return f'upper arm {word(ua):>8} ({ua:+6.1f} deg), ' \
           f'forearm {word(fa):>8} ({fa:+6.1f} deg)'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--shape', default=None,
                    choices=['seven', 'retracted', 'zero', 'all'])
    args = ap.parse_args()

    print(f'model: {os.path.relpath(XACRO, REPO)}')
    print('sagittal-plane geometry. segment angle 0 = pointing forward (+y),')
    print('90 = pointing up (+z). j1 is held at 0, it only yaws the whole arm.')
    print()

    if args.shape in (None, 'zero', 'all'):
        a = analyse(0.0, 0.0)
        print('URDF zero pose [0,0,0,0,0,0]:')
        print(f'  {describe(a)}')
        print(f'  upper arm length {a["upper_len"]:.4f} m, '
              f'forearm length {a["fore_len"]:.4f} m')
        print(f'  TCP = ({a["tcp"][0]:+.3f}, {a["tcp"][1]:+.3f}, {a["tcp"][2]:+.3f})')
        print()

    if args.shape in (None, 'all'):
        # sweep to find the shapes of interest
        print('sweep over j2/j3, reporting the requested shapes:')
        found_seven, found_retract = [], []
        for dj2 in range(-90, 91, 5):
            for dj3 in range(-90, 91, 5):
                a = analyse(float(dj2), float(dj3))
                if a is None:
                    continue
                ua, fa = a['upper_ang'], a['fore_ang']
                # 7 pose: upper arm vertical up, forearm horizontal forward
                if ua > 75 and abs(fa) < 15:
                    found_seven.append((dj2, dj3, ua, fa, a['tcp']))
                # retracted: upper arm forward-ish or down, forearm down
                if abs(ua) < 20 and fa < -60:
                    found_retract.append((dj2, dj3, ua, fa, a['tcp']))

        def show(title, rows):
            print(f'  {title}: {len(rows)} match(es)')
            for dj2, dj3, ua, fa, tcp in rows[:8]:
                print(f'    j2={dj2:+4d} j3={dj3:+4d}  upper {ua:+6.1f} '
                      f'fore {fa:+6.1f}  TCP=({tcp[0]:+.3f},{tcp[1]:+.3f},'
                      f'{tcp[2]:+.3f})')
            print()

        show('upper arm UP + forearm FORWARD  (the 7 pose)', found_seven)
        show('upper arm FORWARD + forearm DOWN (candidate retracted)',
             found_retract)

    if args.shape == 'seven':
        print('searching for upper arm UP + forearm FORWARD:')
        for dj2 in range(-90, 91, 1):
            for dj3 in range(-90, 91, 1):
                a = analyse(float(dj2), float(dj3))
                if a is None:
                    continue
                if a['upper_ang'] > 85 and abs(a['fore_ang']) < 3:
                    print(f'  j2={dj2:+4d} j3={dj3:+4d}  '
                          f'upper {a["upper_ang"]:+6.1f} '
                          f'fore {a["fore_ang"]:+6.1f}  '
                          f'TCP=({a["tcp"][0]:+.3f},{a["tcp"][1]:+.3f},'
                          f'{a["tcp"][2]:+.3f})')
        return 0

    if args.shape == 'retracted':
        print('searching for upper arm FORWARD + forearm DOWN:')
        for dj2 in range(-90, 91, 5):
            for dj3 in range(-90, 91, 5):
                a = analyse(float(dj2), float(dj3))
                if a is None:
                    continue
                if abs(a['upper_ang']) < 12 and a['fore_ang'] < -75:
                    print(f'  j2={dj2:+4d} j3={dj3:+4d}  '
                          f'upper {a["upper_ang"]:+6.1f} '
                          f'fore {a["fore_ang"]:+6.1f}  '
                          f'TCP=({a["tcp"][0]:+.3f},{a["tcp"][1]:+.3f},'
                          f'{a["tcp"][2]:+.3f})')
        return 0

    return 0


if __name__ == '__main__':
    sys.exit(main())
