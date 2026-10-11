#!/usr/bin/env python3
"""Forward kinematics for the Dummy arm URDF. Standard library only.

Why this exists: several questions about this arm are purely geometric and
should not be answered from memory or intuition:

  * What does "all joints zero" actually look like?
  * What is the "7" pose, and what is each joint's angle there?
  * Does the URDF zero correspond to the device zero?

This script parses the xacro, builds the kinematic chain from base_link, and
prints link positions for given joint angles. It also renders a crude ASCII
side view (x-z plane) so a pose can be checked at a glance.

Angles are given in DEGREES on the command line, matching the device
convention, and converted to radians internally.

Usage:
    python tools/fk_dummy.py
    python tools/fk_dummy.py --pose zero
    python tools/fk_dummy.py --pose seven
    python tools/fk_dummy.py --deg 0,0,90,0,0,0
    python tools/fk_dummy.py --deg 0,0,90,0,0,0 --plot
"""
import argparse
import math
import os
import re
import xml.etree.ElementTree as ET

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_XACRO = os.path.join(
    REPO_ROOT, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf', 'dummy2_urdf.xacro')

# Named poses, in URDF degrees.
POSES = {
    'zero': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'seven': [0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
    'reset': [0.0, -75.0, 180.0, 0.0, 0.0, 0.0],
    'home': [0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
}


# ---------------------------------------------------------------------------
# small 4x4 matrix helpers (row-major)
# ---------------------------------------------------------------------------
def mat_identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def mat_from_xyz_rpy(xyz, rpy):
    """URDF semantics: translate by xyz, then rotate by fixed-axis rpy (RPY)."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    # R = Rz(y) * Ry(p) * Rx(r)
    rot = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]
    m = mat_identity()
    for i in range(3):
        for j in range(3):
            m[i][j] = rot[i][j]
        m[i][3] = xyz[i]
    return m


def mat_from_axis_angle(axis, angle):
    """Rodrigues rotation about a (not necessarily unit) axis."""
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12:
        return mat_identity()
    ax, ay, az = ax / n, ay / n, az / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c

    m = mat_identity()
    m[0][0] = ax * ax * C + c
    m[0][1] = ax * ay * C - az * s
    m[0][2] = ax * az * C + ay * s
    m[1][0] = ay * ax * C + az * s
    m[1][1] = ay * ay * C + c
    m[1][2] = ay * az * C - ax * s
    m[2][0] = az * ax * C - ay * s
    m[2][1] = az * ay * C + ax * s
    m[2][2] = az * az * C + c
    return m


# ---------------------------------------------------------------------------
def _safe_eval(expr, props):
    """Evaluate a xacro ${...} expression with the properties we know.

    Returns None when the expression references something unknown, so callers
    can report it instead of silently guessing.
    """
    names = set(re.findall(r'[A-Za-z_][A-Za-z0-9_]*', expr))
    unknown = names - set(props)
    if unknown:
        return None
    try:
        return float(eval(expr, {'__builtins__': {}}, dict(props)))  # noqa: S307
    except Exception:  # noqa: BLE001
        return None


def _resolve(text, props, what, name, missing):
    """Resolve ${...} occurrences in an attribute value."""
    if text is None:
        return None
    if '${' not in text:
        return text

    def repl(m):
        val = _safe_eval(m.group(1), props)
        if val is None:
            missing.append((what, name, m.group(0)))
            return '0'
        return repr(val)

    return re.sub(r'\$\{([^}]*)\}', repl, text)


def _num_list(text):
    return [float(v) for v in text.split()]


def parse_model(path):
    """Return (links, joints) from a .xacro without needing ROS.

    xacro properties are substituted where possible (the URDF uses them for
    the TCP and camera offsets). Anything unresolved is reported rather than
    silently treated as zero, so a wrong FK result cannot pass unnoticed.
    """
    text = open(path, 'r', encoding='utf-8').read()

    props = {}
    if path.endswith('.xacro'):
        # collect <xacro:property name="..." value="..."/> first
        for m in re.finditer(
                r'<xacro:property\s+name="([^"]+)"\s+value="([^"]*)"', text):
            pname, pval = m.group(1), m.group(2)
            v = _safe_eval(pval, props) if '${' in pval else None
            try:
                props[pname] = float(pval) if v is None else v
            except (TypeError, ValueError):
                props[pname] = 0.0

        text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
        text = re.sub(r'<xacro:[^>]*?/>', '', text, flags=re.S)
        text = re.sub(r'<xacro:[^>]*?>.*?</xacro:[^>]*?>', '', text, flags=re.S)

    root = ET.fromstring(text)
    links = {}
    joints = []
    missing = []
    for el in root:
        tag = el.tag.split('}', 1)[-1]
        if tag == 'link':
            links[el.get('name')] = True
        elif tag == 'joint':
            jname = el.get('name')
            origin = el.find('origin')
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                x = _resolve(origin.get('xyz'), props, 'origin.xyz', jname, missing)
                if x:
                    xyz = _num_list(x)
                r = _resolve(origin.get('rpy'), props, 'origin.rpy', jname, missing)
                if r:
                    rpy = _num_list(r)
            axis_el = el.find('axis')
            axis = [1.0, 0.0, 0.0]
            if axis_el is not None and axis_el.get('xyz'):
                a = _resolve(axis_el.get('xyz'), props, 'axis.xyz', jname, missing)
                if a:
                    axis = _num_list(a)
            parent = el.find('parent')
            child = el.find('child')
            limits = None
            lim_el = el.find('limit')
            if lim_el is not None:
                try:
                    limits = (float(lim_el.get('lower')), float(lim_el.get('upper')))
                except (TypeError, ValueError):
                    limits = None
            joints.append({
                'name': jname,
                'type': el.get('type'),
                'parent': parent.get('link') if parent is not None else None,
                'child': child.get('link') if child is not None else None,
                'xyz': xyz,
                'rpy': rpy,
                'axis': axis,
                'limits': limits,
            })

    if missing:
        print('WARNING: unresolved xacro expressions were treated as 0:')
        for what, jname, expr in missing:
            print(f'    joint "{jname}" {what}: {expr}')
        print()

    return links, joints


def chain_to(joints, target):
    """Ordered joint list from the root down to the link named `target`."""
    by_child = {j['child']: j for j in joints}
    chain = []
    cur = target
    guard = 0
    while cur in by_child and guard < 64:
        j = by_child[cur]
        chain.append(j)
        cur = j['parent']
        guard += 1
    chain.reverse()
    return chain, cur


def fk(joints, target, angles_by_joint):
    """Return {link: (x, y, z)} for every link on the chain to `target`."""
    chain, root = chain_to(joints, target)
    T = mat_identity()
    out = {root: (0.0, 0.0, 0.0)}
    for j in chain:
        T = mat_mul(T, mat_from_xyz_rpy(j['xyz'], j['rpy']))
        if j['type'] in ('revolute', 'continuous'):
            T = mat_mul(T, mat_from_axis_angle(j['axis'],
                                               angles_by_joint.get(j['name'], 0.0)))
        out[j['child']] = (T[0][3], T[1][3], T[2][3])
    return out, T


def plot_xz(points, width=52, height=20):
    """Crude side view in the x-z plane. x to the right, z up."""
    pts = [(n, p) for n, p in points.items()]
    xs = [p[0] for _, p in pts]
    zs = [p[2] for _, p in pts]
    x0, x1 = min(xs + [0.0]), max(xs + [0.0])
    z0, z1 = min(zs + [0.0]), max(zs + [0.0])
    dx = max(x1 - x0, 1e-6)
    dz = max(z1 - z0, 1e-6)

    grid = [[' '] * width for _ in range(height)]
    for name, (x, y, z) in pts:
        col = int((x - x0) / dx * (width - 1))
        row = height - 1 - int((z - z0) / dz * (height - 1))
        grid[row][col] = '*'

    print('    side view, x to the right, z up   '
          f'(x {x0:.2f}..{x1:.2f} m, z {z0:.2f}..{z1:.2f} m)')
    for row in grid:
        print('    |' + ''.join(row))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--xacro', default=DEFAULT_XACRO)
    ap.add_argument('--pose', choices=sorted(POSES), default=None)
    ap.add_argument('--deg', default=None,
                    help='six comma-separated joint angles in degrees')
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--all', action='store_true',
                    help='print every named pose')
    args = ap.parse_args()

    links, joints = parse_model(args.xacro)
    revolute = [j for j in joints if j['type'] in ('revolute', 'continuous')]
    revolute.sort(key=lambda j: j['name'])
    names = [j['name'] for j in revolute]

    print(f'model: {os.path.relpath(args.xacro, REPO_ROOT)}')
    print(f'links={len(links)} joints={len(joints)} movable={len(revolute)}')
    print()
    print('joint order used below:')
    for i, n in enumerate(names):
        j = next(x for x in revolute if x['name'] == n)
        print(f'  [{i}] {n:<18} axis={j["axis"]}  '
              f'origin={[round(v, 4) for v in j["xyz"]]}')
    print()

    targets = []
    if args.all or (args.pose is None and args.deg is None):
        targets = [(k, v) for k, v in sorted(POSES.items())]
    elif args.pose is not None:
        targets = [(args.pose, POSES[args.pose])]
    else:
        vals = [float(v) for v in args.deg.split(',')]
        if len(vals) != 6:
            print('--deg needs exactly 6 values')
            return 1
        targets = [('custom', vals)]

    for label, deg in targets:
        print('=' * 70)
        print(f'pose "{label}"  degrees = {deg}')
        print('=' * 70)
        angles = {names[i]: math.radians(deg[i]) for i in range(6)}

        pts, T = fk(joints, 'gripper_tcp', angles)
        order = [n for n in pts if n in ('base_link', 'link1_v1_1',
                                         'link2_3_v4_1', 'link4_v3_1',
                                         'link5_v3_1', 'link6_v3_1',
                                         'link7_v1_1', 'tool0', 'gripper_tcp')]
        print('  link positions (m), base_link at origin:')
        for n in order:
            x, y, z = pts[n]
            print(f'    {n:<18} x={x:+.4f}  y={y:+.4f}  z={z:+.4f}')
        print(f'  gripper_tcp = ({T[0][3]:+.4f}, {T[1][3]:+.4f}, {T[2][3]:+.4f})')
        print(f'  reach (horizontal dist from base) = '
              f'{math.hypot(T[0][3], T[1][3]):.4f} m')
        print(f'  height above base = {T[2][3]:+.4f} m')

        # limit check against the URDF joint limits
        bad = []
        for i, n in enumerate(names):
            j = next(x for x in revolute if x['name'] == n)
            if not j['limits']:
                continue
            lo, hi = j['limits']
            a = math.radians(deg[i])
            tol = 1e-3
            if a < lo - tol or a > hi + tol:
                bad.append(f'{n}={deg[i]:.1f} deg '
                           f'(limit {math.degrees(lo):.1f}..{math.degrees(hi):.1f} deg)')
        print('  limits: ' + ('OK' if not bad else 'VIOLATED -> ' + '; '.join(bad)))
        if args.plot or args.all:
            print()
            plot_xz({n: pts[n] for n in order})
        print()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
