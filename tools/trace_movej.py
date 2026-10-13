#!/usr/bin/env python3
"""Trace exactly how a move_j value is produced, step by step.

Answers "where do the numbers we send actually come from" with the real
formulas from the driver, applied to the values from an actual test run so the
arithmetic can be checked by hand.

The chain has three spaces:

    URDF      what ROS speaks -- radians, and what a trajectory contains
    angle     the device's raw readback, degrees (robot.joint_N.angle)
    move_j    the device's command space, degrees (robot.move_j(...))

with two conversions:

    angle  ->  URDF      urdf_deg  = sign * (angle - joint_offset_deg)
    URDF   ->  move_j    move_j    = urdf_deg / sign + move_j_offset_deg

and the driver's currently installed normalisation of the result.

Usage:
    python tools/trace_movej.py
    python tools/trace_movej.py --urdf 20,0,0,0,0,0 --current 0.22,0.43,-1.47,0.91,-0.04,-0.04
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'config', 'dummy_controller.yaml')

# Defaults taken from a real run:
#   test_motion.sh 1 20 read this pose and added 20 deg to joint 1.
URDF_TARGET = [20.22, 0.43, -1.47, 0.91, -0.04, -0.04]
CURRENT_URDF = [0.22, 0.43, -1.47, 0.91, -0.04, -0.04]


def get_list(text, key):
    m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
    return [float(v) for v in m.group(1).split(',')] if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', default=None,
                    help='target URDF pose in degrees, comma separated')
    ap.add_argument('--current', default=None,
                    help='current URDF pose in degrees, comma separated')
    args = ap.parse_args()

    target = ([float(v) for v in args.urdf.split(',')] if args.urdf
              else URDF_TARGET)
    current = ([float(v) for v in args.current.split(',')] if args.current
               else CURRENT_URDF)

    text = open(CONFIG, encoding='utf-8').read()
    sign = get_list(text, 'joint_sign')
    joff = get_list(text, 'joint_offset_deg')
    mjoff = get_list(text, 'move_j_offset_deg')
    print('parameters from config/dummy_controller.yaml')
    print(f'  joint_sign       = {sign}')
    print(f'  joint_offset_deg = {joff}')
    print(f'  move_j_offset_deg= {mjoff}')
    print()

    def urdf_to_movej_deg(u):
        """The driver's two-step chain, in degrees, no normalisation."""
        out = []
        for i in range(6):
            s = sign[i] if abs(sign[i]) > 1e-12 else 1.0
            # angle = urdf/sign + joint_offset_deg
            angle = u[i] / s + joff[i]
            # move_j = angle + move_j_offset_deg
            out.append(angle + mjoff[i])
        return out

    print('STEP 1 -- where the target came from')
    print(f'  test_motion.sh read the current pose and added 20 deg to joint_1:')
    print(f'    current URDF = {current}')
    print(f'    target  URDF = {target}   (joint_1 +20)')
    print()

    print('STEP 2 -- URDF -> angle -> move_j, per joint')
    print(f'  {"joint":<7} {"URDF":>9} {"/sign":>9} {"+joff":>10} '
          f'{"= angle":>10} {"+mjoff":>9} {"= move_j":>10}')
    raw_mj = urdf_to_movej_deg(target)
    for i in range(6):
        s = sign[i] if abs(sign[i]) > 1e-12 else 1.0
        a1 = target[i] / s
        a2 = a1 + joff[i]
        a3 = a2 + mjoff[i]
        print(f'  joint_{i + 1:<1} {target[i]:>9.2f} {a1:>9.2f} {joff[i]:>10.3f} '
              f'{a2:>10.2f} {mjoff[i]:>9.2f} {a3:>10.2f}')
    print()
    print(f'  raw move_j (before any normalisation) = '
          f'{[round(v, 2) for v in raw_mj]}')
    print()

    print('STEP 3 -- the nearest-whole-turn step, where j3 breaks')
    print()
    print('  Careful: the driver derives the current pose with a DIFFERENT')
    print('  formula from the one it uses for the target:')
    print('      current_move_j = raw_readback - move_j_offset_deg')
    print('  That is the directly measured relation (commanding move_j j3 = 90')
    print('  puts the raw readback at about -90, so that joint\'s offset is')
    print('  about -180). The target instead comes from the fitted chain in')
    print('  STEP 2, which for j3 implies an offset of -91.474. The two')
    print('  disagree, and that disagreement is what pushes the answer out of')
    print('  the accepted range.')
    print()

    raw_readback = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
    cur_mj = [raw_readback[i] - mjoff[i] for i in range(6)]
    print(f'  raw readback at the 7 pose = {raw_readback}')
    print(f'  current pose in move_j     = raw - mjoff = '
          f'{[round(v, 2) for v in cur_mj]}')
    print()
    print(f'  {"joint":<7} {"raw":>10} {"current":>10} {"normalised":>12} '
          f'{"way":>12}')
    norm = []
    for i in range(6):
        t = raw_mj[i]
        c = cur_mj[i]
        # what the driver now does: nearest whole-turn equivalent
        best = t + 360.0 * round((c - t) / 360.0)
        norm.append(best)
        turns = round((best - t) / 360.0)
        way = 'raw' if turns == 0 else f'{turns:+d} turn'
        print(f'  joint_{i + 1:<1} {t:>10.2f} {c:>10.2f} {best:>12.2f} '
              f'{way:>12}')
    print()
    print(f'  final move_j sent = {[round(v, 2) for v in norm]}')
    print()

    print('=' * 62)
    print('So -268.53 is not an arithmetic slip. It is the raw value for the')
    print('pose you asked for, shifted one whole turn to sit near where the')
    print('driver believes j3 currently is. The driver assumes whole-turn')
    print('shifts are interchangeable, and the j3 sweep proves they are not:')
    print('90 and 91.474 were accepted, 270 and -268.526 were refused.')
    print()
    print('Two things need fixing:')
    print('  1. the whole-turn shift must be constrained to the range the')
    print('     device actually accepts, per joint -- tools/probe_range.py')
    print('     measures that range;')
    print('  2. j3\'s calibration disagrees between its two uses (target chain')
    print('     implies -91.474, measured readback implies about -180). Settle')
    print('     that before trusting any j3 target.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
