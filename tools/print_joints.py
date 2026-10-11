#!/usr/bin/env python3
"""Print the arm's joints and their URDF limits, in degrees.

Usage:
    python3 tools/print_joints.py
"""
import math
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')


def main():
    text = open(XACRO, encoding='utf-8').read()
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)

    print('Joints in the model, with URDF limits in degrees:')
    print(f'  {"joint":<20} {"type":<10} {"lower":>9} {"upper":>9}')
    for m in re.finditer(r'<joint\s+name="([^"]+)"[^>]*>(.*?)</joint>',
                         text, re.S):
        name, body = m.group(1), m.group(2)
        type_m = re.search(r'type="([^"]+)"', m.group(0))
        jtype = type_m.group(1) if type_m else '?'
        if jtype not in ('revolute', 'prismatic', 'continuous'):
            continue
        lm = re.search(r'<limit\s+([^>]*)/>', body)
        if not lm:
            print(f'  {name:<20} {jtype:<10} {"--":>9} {"--":>9}')
            continue
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', lm.group(1)))
        try:
            lo = math.degrees(float(attrs['lower']))
            hi = math.degrees(float(attrs['upper']))
            print(f'  {name:<20} {jtype:<10} {lo:>+9.1f} {hi:>+9.1f}')
        except (KeyError, ValueError):
            print(f'  {name:<20} {jtype:<10} {"?":>9} {"?":>9}')

    print()
    print('NOTE: these limits come from the URDF. They are known to be')
    print('      narrower than the real arm in places: the retracted pose')
    print('      needs j3 = -90, which sits exactly on the boundary, and the')
    print('      driver commands a retract pose that maps beyond it. Do not')
    print('      treat these as hard mechanical stops.')
    print()
    print('      The driver does NOT enforce these limits when executing a')
    print('      trajectory, so an out-of-range target will still be sent.')
    print('      Keep targets inside the range until MoveIt is configured.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
