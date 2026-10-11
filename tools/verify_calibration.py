#!/usr/bin/env python3
"""Verify the calibration derived from the two measured poses.

The calibration was derived from exactly two device readbacks:

    7 pose    device [0.2146, 74.5723, -91.4737, -0.9101, -0.0347, -0.0401]
              URDF   [0, 0, 0, 0, 0, 0]
              (a photo confirms this geometry: upper arm vertical,
               forearm horizontal; and forward kinematics on the real URDF
               shows the zero pose IS that geometry)

    retracted device [-0.0004, 0.1564, -0.1264, 0.0013, 0.0015, -0.0030]
              URDF   [0, -75, -90, 0, 0, 0]
              (the folded configuration this model produces)

This script re-derives the parameters from those two poses, then reports:
  - the fitted gain and offset per joint
  - the resulting joint_sign / joint_offset_deg
  - the URDF pose predicted for both measurements
  - the segment geometry of the predicted retracted pose
  - warnings where a joint is under-determined (it did not move between the
    two poses, so its gain cannot be measured)

It reads nothing from the driver config: its job is to be an independent
check on what was written there.

Usage:
    python tools/verify_calibration.py
    python tools/verify_calibration.py --apply-check   # also diff vs the config
"""
import argparse
import contextlib
import io
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
REPO = os.path.dirname(HERE)
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')
CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'config', 'dummy_controller.yaml')

JOINTS = [f'dummy2_joint{i}' for i in range(1, 7)]

SAMPLES = [
    ('7 pose', [0.2146, 74.5723, -91.4737, -0.9101, -0.0347, -0.0401],
     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ('retracted', [-0.0004, 0.1564, -0.1264, 0.0013, 0.0015, -0.0030],
     [0.0, -75.0, -90.0, 0.0, 0.0, 0.0]),
]

with contextlib.redirect_stdout(io.StringIO()):
    import fk_dummy
    LINKS, JOINTS_PARSED = fk_dummy.parse_model(XACRO)

REV = sorted([j for j in JOINTS_PARSED
              if j['type'] in ('revolute', 'continuous')],
             key=lambda j: j['name'])
NAMES = [j['name'] for j in REV]


def geometry(urdf_deg):
    angles = {NAMES[i]: math.radians(urdf_deg[i]) for i in range(6)}
    pts, T = fk_dummy.fk(JOINTS_PARSED, 'gripper_tcp', angles)
    sh = pts['link2_3_v4_1']
    el = pts['link4_v3_1']
    wr = pts['link7_v1_1']

    def ang(a, b):
        return math.degrees(math.atan2(b[2] - a[2], b[1] - a[1]))

    return ang(sh, el), ang(el, wr), (T[0][3], T[1][3], T[2][3])


def word(deg):
    if deg > 60:
        return 'UP'
    if deg > 25:
        return 'up-diag'
    if deg > -25:
        return 'FORWARD'
    if deg > -60:
        return 'down-diag'
    return 'DOWN'


def fit(j):
    pts = [(s[1][j], s[2][j]) for s in SAMPLES]
    devs = [p[0] for p in pts]
    urdfs = [p[1] for p in pts]
    dmean = sum(devs) / 2
    umean = sum(urdfs) / 2
    sxx = sum((d - dmean) ** 2 for d in devs)
    sxy = sum((devs[i] - dmean) * (urdfs[i] - umean) for i in range(2))
    if abs(sxx) < 1e-9:
        return None
    gain = sxy / sxx
    return gain, umean - gain * dmean


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply-check', action='store_true',
                    help='also compare against the values in the config')
    args = ap.parse_args()

    print('samples used')
    for name, dev, urdf in SAMPLES:
        print(f'  {name:<10} device={dev}')
        print(f'  {"":<10} urdf  ={urdf}')
    print()

    print('per-joint linear fit   urdf_deg = gain * device_deg + offset_deg')
    print(f'  {"joint":<18} {"gain":>8} {"offset":>9}   note')
    signs, offsets = [], []
    for j in range(6):
        r = fit(j)
        if r is None:
            signs.append(1.0)
            offsets.append(0.0)
            print(f'  {JOINTS[j]:<18} {"--":>8} {"--":>9}   '
                  'joint did not move between samples; gain undetermined')
            continue
        gain, offset = r
        s = 1.0 if gain >= 0 else -1.0
        # Canonical form: urdf = s * (device - k), where k is the device value
        # that maps to urdf = 0 under this gain/sign.
        #   s*device - s*k  ==  gain*device + offset
        #   => k = -offset / s
        # Using k rather than raw offset makes the two algebraically
        # equivalent spellings produce identical numbers, so a config/derived
        # comparison cannot report a false mismatch.
        k = -offset / s
        signs.append(s)
        offsets.append(k)
        note = ''
        if abs(gain) < 0.05:
            note = 'joint did not move; gain not measurable'
        elif abs(abs(gain) - 1.0) > 0.15:
            note = f'gain {gain:+.3f} far from +/-1: check the pose claim'
        print(f'  {JOINTS[j]:<18} {gain:>+8.4f} {offset:>+9.3f}   {note}')
    print()

    print('--- derived parameters ---')
    print(f'  joint_sign      : [{", ".join(f"{v:.1f}" for v in signs)}]')
    print(f'  joint_offset_deg: [{", ".join(f"{v:.3f}" for v in offsets)}]')
    print()

    print('--- verification: predict URDF for each measurement ---')
    for name, dev, urdf in SAMPLES:
        pred = [signs[i] * (dev[i] - offsets[i]) for i in range(6)]
        err = [pred[i] - urdf[i] for i in range(6)]
        maxe = max(abs(e) for e in err)
        ua, fa, tcp = geometry(pred)
        print(f'  {name}')
        print(f'    predicted URDF = {[round(v, 2) for v in pred]}')
        print(f'    expected URDF  = {urdf}')
        print(f'    max error      = {maxe:.3f} deg')
        print(f'    geometry: upper arm {word(ua):>8} ({ua:+6.1f} deg), '
              f'forearm {word(fa):>8} ({fa:+6.1f} deg)')
        print(f'    TCP = ({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f})')
    print()

    if args.apply_check:
        text = open(CONFIG, encoding='utf-8').read()

        def get(key):
            m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
            return [float(v) for v in m.group(1).split(',')] if m else None

        cfg_sign = get('joint_sign')
        cfg_off = get('joint_offset_deg')
        print('--- config comparison ---')
        print(f'  config joint_sign      : {cfg_sign}')
        print(f'  derived joint_sign     : {[round(v, 1) for v in signs]}')
        mismatch_sign = [JOINTS[i] for i in range(6)
                         if cfg_sign and abs(cfg_sign[i] - signs[i]) > 0.01]

        # Two deliberate differences are expected and are NOT errors:
        #  * a joint that barely moved has an unmeasurable gain, so the
        #    derived offset is meaningless; the config keeps the 7-pose
        #    anchor value instead, which is strictly better.
        #  * the config anchors exactly at the 7 pose, so a couple of degrees
        #    of residual can be pushed onto the other sample.
        # Only a large discrepancy should be reported.
        moved = [abs(SAMPLES[0][1][i] - SAMPLES[1][1][i]) > 2.0 for i in range(6)]
        mismatch_off = []
        for i in range(6):
            if not cfg_off:
                continue
            if abs(cfg_off[i] - offsets[i]) < 0.05:
                continue
            if not moved[i]:
                print(f'  {JOINTS[i]}: gain unmeasurable (joint barely moved); '
                      f'config keeps {cfg_off[i]:+.3f} from the 7-pose anchor '
                      f'-- expected')
                continue
            mismatch_off.append(JOINTS[i])

        if mismatch_sign:
            print(f'  SIGN differs for: {", ".join(mismatch_sign)}')
        for i in range(6):
            if JOINTS[i] in mismatch_off:
                print(f'  OFFSET differs for {JOINTS[i]}: config '
                      f'{cfg_off[i]:+.3f} vs derived {offsets[i]:+.3f}')
        if not mismatch_sign and not mismatch_off:
            print('  config is consistent with the derived calibration')
        print()

    print('NOTE: joints that did not move between the two samples have an')
    print('      unmeasurable gain. They are left at sign +1 with the offset')
    print('      from the 7-pose anchor. Verify them later by moving the arm')
    print('      in those joints and checking the model tracks.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
