#!/usr/bin/env python3
"""Convert device joint readings (degrees) to URDF joint angles, and more.

Three modes, all using the same conversion and the same URDF limits:

  1. device -> urdf   What pose is the arm actually in?
  2. urdf   -> device What will the device read if I command this pose?
  3. pose names       Check the configured named poses (zero / home / reset)

The offsets live in the driver config. This script reads the same file so the
two can never drift apart.

Units: device side is DEGREES, URDF side is RADIANS (ROS convention).
Conversion:  urdf_deg = sign * device_deg + offset_deg

Usage:
    python tools/dev2urdf.py --device 0.03,76.2,-93.8,-0.03,0.18,-0.03
    python tools/dev2urdf.py --urdf-deg 0,75,0,0,0,0
    python tools/dev2urdf.py --poses
    python tools/dev2urdf.py --device 0,0,0,0,0,0 --plot
"""
import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_ros2_driver',
                      'config', 'dummy_controller.yaml')
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')

JOINT_NAMES = [f'dummy2_joint{i}' for i in range(1, 7)]


def load_params(path=CONFIG):
    """Minimal YAML reader for the flat lists we need. No PyYAML dependency."""
    text = open(path, 'r', encoding='utf-8').read()

    def get_list(key, default):
        m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
        if not m:
            return default
        return [float(v) for v in m.group(1).split(',') if v.strip()]

    sign = get_list('joint_sign', [1.0] * 6)
    off = get_list('joint_offset_deg', [0.0] * 6)
    home = get_list('home_joints_deg', [0.0] * 6)
    reset = get_list('reset_joints_deg', [0.0] * 6)
    return {
        'joint_sign': sign,
        'joint_offset_deg': off,
        'home_joints_deg': home,
        'reset_joints_deg': reset,
        'source': path,
    }


def load_limits(path=XACRO):
    """Per-joint (lower, upper) limits in degrees, from the URDF."""
    text = open(path, 'r', encoding='utf-8').read()
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    limits = {}
    for m in re.finditer(
            r'<joint\s+name="(dummy2_joint\d)"[^>]*>(.*?)</joint>',
            text, re.S):
        name, body = m.group(1), m.group(2)
        lm = re.search(r'<limit\s+([^>]*)/>', body)
        if not lm:
            continue
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', lm.group(1)))
        try:
            lo = float(attrs['lower'])
            hi = float(attrs['upper'])
        except (KeyError, ValueError):
            continue
        limits[name] = (math.degrees(lo), math.degrees(hi))
    return limits


def fmt(vals, w=9, prec=3):
    return '[' + ', '.join(f'{v:>{w}.{prec}f}' for v in vals) + ']'


def check_limits(urdf_deg, limits, names=JOINT_NAMES):
    bad = []
    for i, n in enumerate(names):
        if n not in limits:
            continue
        lo, hi = limits[n]
        if urdf_deg[i] < lo - 0.5 or urdf_deg[i] > hi + 0.5:
            bad.append(f'{n}={urdf_deg[i]:.1f} (limit {lo:.0f}..{hi:.0f})')
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', help='six device readings, degrees, comma separated')
    ap.add_argument('--urdf-deg', dest='urdf_deg',
                    help='six URDF angles in degrees; prints expected device values')
    ap.add_argument('--poses', action='store_true',
                    help='check the named poses from the config')
    ap.add_argument('--plot', action='store_true')
    args = ap.parse_args()

    p = load_params()
    sign = p['joint_sign']
    off = p['joint_offset_deg']
    limits = load_limits()

    print('params from ' + os.path.relpath(p['source'], REPO))
    print('  joint_sign       = ' + str(sign))
    print('  joint_offset_deg = ' + str(off))
    print('  conversion       = urdf_deg = sign * device_deg + offset_deg')
    print()

    def device_to_urdf_deg(d):
        return [sign[i] * d[i] + off[i] for i in range(6)]

    def urdf_deg_to_device(u):
        return [(u[i] - off[i]) / (sign[i] if abs(sign[i]) > 1e-12 else 1.0)
                for i in range(6)]

    if args.poses or (args.device is None and args.urdf_deg is None):
        for label, key in (('zero (all zeros)', None),
                           ('home (from config)', 'home_joints_deg'),
                           ('reset (from config)', 'reset_joints_deg')):
            if key is None:
                u = [0.0] * 6
            else:
                u = p[key]
            d = urdf_deg_to_device(u)
            bad = check_limits(u, limits)
            print(f'pose "{label}"')
            print('  URDF deg  = ' + fmt(u))
            print('  URDF rad  = ' + fmt([math.radians(v) for v in u]))
            print('  device deg= ' + fmt(d))
            print('  limits    : ' + ('OK' if not bad
                                      else 'VIOLATED -> ' + '; '.join(bad)))
            print()
        return 0

    if args.device:
        d = [float(v) for v in args.device.split(',')]
        if len(d) != 6:
            print('--device needs exactly 6 values')
            return 1
        u = device_to_urdf_deg(d)
        bad = check_limits(u, limits)
        print('device -> URDF')
        print('  device deg= ' + fmt(d))
        print('  URDF  deg = ' + fmt(u))
        print('  URDF  rad = ' + fmt([math.radians(v) for v in u]))
        print('  limits    : ' + ('OK' if not bad
                                  else 'VIOLATED -> ' + '; '.join(bad)))
        print()
        print('  per-joint detail:')
        for i, n in enumerate(JOINT_NAMES):
            lo, hi = limits.get(n, (float('nan'), float('nan')))
            print(f'    {n:<16} device={d[i]:+9.3f}  urdf={u[i]:+9.3f} deg'
                  f'  ({math.radians(u[i]):+7.4f} rad)  limit {lo:.0f}..{hi:.0f}')

        if args.plot:
            print()
            try:
                import fk_dummy
                links, joints = fk_dummy.parse_model(XACRO)
                rev = sorted([j for j in joints
                              if j['type'] in ('revolute', 'continuous')],
                             key=lambda j: j['name'])
                names = [j['name'] for j in rev]
                angles = {names[i]: math.radians(u[i]) for i in range(6)}
                pts, T = fk_dummy.fk(joints, 'gripper_tcp', angles)
                fk_dummy.plot_xz(pts)
                print(f'    gripper_tcp = ({T[0][3]:+.4f}, {T[1][3]:+.4f}, '
                      f'{T[2][3]:+.4f})')
            except Exception as exc:  # noqa: BLE001
                print(f'    (plot unavailable: {exc})')
        return 0

    u = [float(v) for v in args.urdf_deg.split(',')]
    if len(u) != 6:
        print('--urdf-deg needs exactly 6 values')
        return 1
    d = urdf_deg_to_device(u)
    bad = check_limits(u, limits)
    print('URDF -> device (what the arm will be commanded to read)')
    print('  URDF  deg = ' + fmt(u))
    print('  device deg= ' + fmt(d))
    print('  limits    : ' + ('OK' if not bad
                              else 'VIOLATED -> ' + '; '.join(bad)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
