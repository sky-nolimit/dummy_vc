#!/usr/bin/env python3
"""Calibration solver for the Dummy arm: device angles -> URDF joint angles.

The problem
-----------
Two coordinate spaces describe the same joint, and the mapping between them is
not known a priori:

    device angle  (robot.joint_N.angle)   degrees, raw encoder readback
    URDF angle    (what the model needs)  radians

For each joint the mapping is assumed affine:

    urdf_deg = gain * device_deg + offset_deg

When gain is 1 this reduces to a pure offset. Do NOT assume gain is 1: the
two spaces can use different mechanical references.

What this script does
---------------------
You supply measured pairs: a device readback plus the URDF pose you believe
the arm is physically in. Give at least two different poses per joint, or the
gain cannot be determined and the solver will say so instead of guessing.

    python3 tools/solve_calibration.py --list
    python3 tools/solve_calibration.py \\
        --sample retract "device=0,0,0,0,0,0"   "urdf=0,-90,0,0,0,0" \\
        --sample seven   "device=0,75,-91,0,0,0" "urdf=0,0,0,0,0,0"

Output is a ready-to-paste joint_offset_deg / joint_sign pair plus the
residual for every joint, so a bad assumption shows up as a large residual
rather than silently producing wrong parameters.

IMPORTANT: the solver can only be as good as the URDF poses you assert. If the
arm was positioned by hand, expect a few degrees of error. The residual column
is there to make that visible.
"""
import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')

JOINTS = [f'dummy2_joint{i}' for i in range(1, 7)]

# Measured device readbacks, from /calib/safety_check (angle space, degrees).
KNOWN_SAMPLES = {
    'retract': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'seven': [0.2146, 74.5723, -91.4737, -0.9101, -0.0347, -0.0401],
}


def parse_pair(text):
    """Parse 'device=a,b,c,d,e,f' or 'urdf=a,b,c,d,e,f'."""
    if '=' not in text:
        raise ValueError(f'expected device=... or urdf=..., got {text!r}')
    key, vals = text.split('=', 1)
    key = key.strip().lower()
    if key not in ('device', 'urdf'):
        raise ValueError(f'unknown key {key!r}')
    nums = [float(v) for v in vals.split(',') if v.strip()]
    if len(nums) != 6:
        raise ValueError(f'{key} needs 6 values, got {len(nums)}')
    return key, nums


def load_limits():
    text = open(XACRO, encoding='utf-8').read()
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    limits = {}
    for m in re.finditer(
            r'<joint\s+name="(dummy2_joint\d)"[^>]*>(.*?)</joint>', text, re.S):
        name, body = m.group(1), m.group(2)
        lm = re.search(r'<limit\s+([^>]*)/>', body)
        if not lm:
            continue
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', lm.group(1)))
        try:
            limits[name] = (math.degrees(float(attrs['lower'])),
                            math.degrees(float(attrs['upper'])))
        except (KeyError, ValueError):
            pass
    return limits


def solve(pairs, joint_index):
    """Least-squares fit urdf = gain*device + offset for one joint.

    Returns (gain, offset, residuals) or None when under-determined.
    """
    pts = [(p['device'][joint_index], p['urdf'][joint_index])
           for p in pairs if 'device' in p and 'urdf' in p]
    if len(pts) < 2:
        return None
    devs = [p[0] for p in pts]
    urdfs = [p[1] for p in pts]

    dmean = sum(devs) / len(devs)
    umean = sum(urdfs) / len(urdfs)
    sxx = sum((d - dmean) ** 2 for d in devs)
    sxy = sum((devs[i] - dmean) * (urdfs[i] - umean) for i in range(len(pts)))

    if abs(sxx) < 1e-9:
        return None  # device did not move -> gain indeterminate
    gain = sxy / sxx
    offset = umean - gain * dmean
    resid = [urdfs[i] - (gain * devs[i] + offset) for i in range(len(pts))]
    return gain, offset, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sample', nargs='+', action='append', metavar='NAME',
                    help='NAME device=... urdf=...  (repeatable)')
    ap.add_argument('--list', action='store_true',
                    help='show the built-in measured device readbacks')
    args = ap.parse_args()

    if args.list or not args.sample:
        print('Built-in measured device readbacks (angle space, degrees):')
        for name, vals in KNOWN_SAMPLES.items():
            print(f'  {name:<10} {vals}')
        print()
        print('To solve, supply poses. Example:')
        print('  python3 tools/solve_calibration.py \\')
        print('    --sample retract "device=0,0,0,0,0,0" "urdf=0,-90,0,0,0,0" \\')
        print('    --sample seven   "device=0,75,-91,0,0,0" "urdf=0,0,0,0,0,0"')
        print()
        print('The urdf= values are YOUR claim about the physical pose.')
        print('A wrong claim shows up as a large residual, not as an error.')
        if not args.sample:
            return 0

    pairs = []
    for group in args.sample:
        name = group[0]
        entry = {'name': name}
        for item in group[1:]:
            key, nums = parse_pair(item)
            entry[key] = nums
        if 'device' not in entry:
            # allow naming a built-in sample
            if name in KNOWN_SAMPLES:
                entry['device'] = KNOWN_SAMPLES[name]
            else:
                print(f'sample "{name}" has no device= and is not a built-in')
                return 1
        pairs.append(entry)

    print('samples')
    for p in pairs:
        print(f'  {p["name"]:<12} device={p.get("device")}')
        print(f'  {"":<12} urdf  ={p.get("urdf", "<not given>")}')
    print()

    limits = load_limits()
    gains, offsets, signs, problems = [], [], [], []

    print('per-joint fit:  urdf_deg = gain * device_deg + offset_deg')
    print(f'  {"joint":<18} {"gain":>9} {"offset":>10} {"resid":>9}  limits')
    for j in range(6):
        res = solve(pairs, j)
        if res is None:
            gains.append(None)
            offsets.append(None)
            signs.append(1.0)
            print(f'  {JOINTS[j]:<18} {"n/a":>9} {"n/a":>10} {"n/a":>9}'
                  '  (needs 2+ poses where this joint moves)')
            continue
        gain, offset, resid = res
        maxr = max(abs(r) for r in resid)
        limits.append if False else None
        lo, hi = limits.get(JOINTS[j], (float("nan"), float("nan")))
        gains.append(gain)
        offsets.append(offset)
        signs.append(1.0 if gain >= 0 else -1.0)
        flag = '' if maxr < 3.0 else '  <-- residual large'
        print(f'  {JOINTS[j]:<18} {gain:>9.4f} {offset:>10.3f} {maxr:>9.3f}'
              f'  {lo:.0f}..{hi:.0f}{flag}')
        if maxr >= 3.0:
            problems.append(f'{JOINTS[j]}: residual {maxr:.2f} deg')
    print()

    if any(g is None for g in gains):
        print('NOTE: some joints are under-determined. Move those joints between')
        print('      samples, or the offsets for them cannot be trusted.')
        print()

    # Emit parameters in the driver's convention:
    #   urdf = sign * (angle - joint_offset_deg) + urdf_offset_deg
    # With urdf_offset_deg = 0 this means
    #   sign * (angle - offset) = gain*angle + off
    # so sign = sign(gain), and offset chosen to match the constant term.
    print('--- paste into config/dummy_controller.yaml ---')
    sign_list, off_list = [], []
    for j in range(6):
        if gains[j] is None:
            sign_list.append(1.0)
            off_list.append(0.0)
            continue
        g = gains[j]
        o = offsets[j]
        s = 1.0 if g >= 0 else -1.0
        # urdf = s*(angle - k)  =>  s*angle - s*k.  Match gain*angle + o.
        # s must equal g's sign; the magnitude of g is assumed 1 here, which the
        # driver cannot express, so report the mismatch when it is large.
        k = -o / s
        sign_list.append(s)
        off_list.append(k)
        if abs(abs(g) - 1.0) > 0.05:
            problems.append(
                f'{JOINTS[j]}: gain {g:.3f} is not 1; the driver assumes unit '
                'gain, so this joint needs code support for gain')

    print(f'  joint_sign      : [{", ".join(f"{v:.1f}" for v in sign_list)}]')
    print(f'  joint_offset_deg: [{", ".join(f"{v:.3f}" for v in off_list)}]')
    print()

    if problems:
        print('=' * 62)
        print('ISSUES FOUND')
        for p in problems:
            print(f'  - {p}')
        print()
        print('A large residual means your asserted URDF pose does not match the')
        print('device reading under a linear model. Either the pose claim is off,')
        print('or the two spaces are not related by a simple affine map.')
        return 1

    print('RESULT: all joints fit with small residuals.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
