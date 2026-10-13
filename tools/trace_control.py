#!/usr/bin/env python3
"""Show the whole control chain, and print every value with its reason.

Answers "I want to rotate j3 by 30 and j5 by 15 -- what gets sent, and why".

Reading this, the single most important idea is how "rotate by 30 degrees" is
expressed. The absolute target depends on where the joint started, so it has to
be built from the current pose. What is absolute is the ROTATION, and the
conversion for a rotation is sign-free:

    a rotation of D degrees of the physical joint
        = D degrees of change in the raw readback
        = D / sign degrees of change in the move_j command
        = D * sign degrees of change in URDF

so the target is

    move_j_target = current_move_j + D / sign

Everything below is that, spelled out.

Usage:
    python tools/trace_control.py
    python tools/trace_control.py --rotate 3:30,5:15
    python tools/trace_control.py --rotate 3:30 --current 0.215,74.572,-91.474,-0.910,-0.035,-0.040
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'config', 'dummy_controller.yaml')

# A real readback: the arm sitting at the 7 pose.
DEFAULT_CURRENT_RAW = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
# What homing() leaves behind (a second real observation, ~1.5 deg different).
HOMING_RAW = [0.0, 75.0, -90.0, 0.0, 0.0, 0.0]


def get_list(text, key):
    m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
    return [float(v) for v in m.group(1).split(',')] if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rotate', default='3:30,5:15',
                    help='joint:degrees pairs, 1-based, comma separated')
    ap.add_argument('--current', default=None,
                    help='current raw readback in degrees, comma separated')
    args = ap.parse_args()

    rotate = {}
    for part in args.rotate.split(','):
        j, d = part.split(':')
        rotate[int(j) - 1] = float(d)
    current_raw = ([float(v) for v in args.current.split(',')]
                   if args.current else DEFAULT_CURRENT_RAW)

    text = open(CONFIG, encoding='utf-8').read()
    sign = get_list(text, 'joint_sign')
    joff = get_list(text, 'joint_offset_deg')
    mjoff = get_list(text, 'move_j_offset_deg')
    mjump = float(re.search(r'^\s*max_move_jump_deg:\s*([-\d.]+)', text,
                            re.M).group(1))

    print('=' * 74)
    print('PARAMETERS (config/dummy_controller.yaml -- the single source)')
    print('=' * 74)
    print(f'  joint_sign        = {sign}')
    print(f'  joint_offset_deg  = {joff}')
    print(f'  move_j_offset_deg = {mjoff}')
    print(f'  max_move_jump_deg = {mjump}')
    print()

    print('=' * 74)
    print('THE THREE SPACES')
    print('=' * 74)
    print('  raw (angle)  robot.joint_N.angle -- what the device reports.')
    print('               Degrees. This is the physical joint, measured.')
    print('  URDF         what ROS speaks: /joint_states, MoveIt, trajectories.')
    print('               Radians on the wire, degrees in the maths here.')
    print('  move_j       the device command space: robot.move_j(a..f).')
    print('               Degrees. NOT the same numbers as the raw readback.')
    print()
    print('  raw -> URDF   urdf_deg = sign * (raw - joint_offset_deg)')
    print('  URDF -> raw   raw_deg  = urdf_deg / sign + joint_offset_deg')
    print('  move_j <-> raw  differ by a per-joint constant, measured:')
    print('                    raw = move_j + move_j_offset_deg')
    print('                  (the driver applies the equivalent of this when it')
    print('                   derives the current pose as raw - move_j_offset)')
    print()

    print('=' * 74)
    print('STEP 0 -- where we are now')
    print('=' * 74)
    print(f'  raw readback from the device = {current_raw}')
    print(f'  convert to URDF (sign * (raw - joint_offset_deg)):')
    urdf_now = [sign[i] * (current_raw[i] - joff[i]) for i in range(6)]
    print(f'    URDF = {[round(v, 3) for v in urdf_now]}')
    print('  the driver derives the current COMMAND pose as raw - move_j_offset:')
    mj_now = [current_raw[i] - mjoff[i] for i in range(6)]
    print(f'    current move_j = {[round(v, 3) for v in mj_now]}')
    print()

    print('=' * 74)
    print('STEP 1 -- what "rotate by D degrees" means')
    print('=' * 74)
    print('  A rotation is a DIFFERENCE, so it is free of origin offsets:')
    print('      D degrees of physical rotation')
    print('        = D degrees of change in the raw readback')
    print('        = D / sign degrees of change in move_j')
    print('        = D * sign degrees of change in URDF')
    print()
    for idx, D in sorted(rotate.items()):
        s = sign[idx] if abs(sign[idx]) > 1e-12 else 1.0
        print(f'    joint_{idx + 1}: rotate {D:+.1f} deg')
        print(f'        URDF change    {D * s:+8.3f}   ({D:+.1f} x sign {s:+.0f})')
        print(f'        move_j change  {D / s:+8.3f}   ({D:+.1f} / sign {s:+.0f})')
    print()

    print('=' * 74)
    print('STEP 2 -- build the absolute target')
    print('=' * 74)
    print('  move_j_target = current_move_j + D / sign   (unchanged joints keep')
    print('  their current value; they are NOT set to zero)')
    print()
    target_mj_raw = list(mj_now)
    for idx, D in rotate.items():
        s = sign[idx] if abs(sign[idx]) > 1e-12 else 1.0
        target_mj_raw[idx] = mj_now[idx] + D / s
    print(f'  {"joint":<8} {"current":>10} {"rotate":>8} {"/sign":>8} '
          f'{"change":>9} {"raw target":>11}')
    for i in range(6):
        D = rotate.get(i, 0.0)
        s = sign[i] if abs(sign[i]) > 1e-12 else 1.0
        ch = D / s
        print(f'  joint_{i + 1:<2} {mj_now[i]:>10.3f} {D:>8.1f} {s:>8.0f} '
              f'{ch:>9.3f} {target_mj_raw[i]:>11.3f}')
    print()
    print(f'  before normalisation: '
          f'{[round(v, 3) for v in target_mj_raw]}')
    print()

    print('=' * 74)
    print('STEP 3 -- normalisation, and THE BUG')
    print('=' * 74)
    print('  The device does NOT fold whole turns together. Measured on j3:')
    print('      90 and 91.474 accepted')
    print('      -268.526, -270, 270, 450 refused')
    print('  A refused value makes the device drop the WHOLE command silently,')
    print('  so the arm simply does not move.')
    print()
    print('  The driver currently shifts each value to the whole-turn')
    print('  equivalent nearest the current pose. When raw-current is close to')
    print('  half a turn the rounding lands on the boundary and shifts anyway:')
    print()
    print(f'  {"joint":<8} {"target":>10} {"current":>10} {"shifted":>10} '
          f'{"turns":>7} {"travel":>9} {"within":>7}')
    final = []
    for i in range(6):
        t = target_mj_raw[i]
        c = mj_now[i]
        best = t + 360.0 * round((c - t) / 360.0)
        turns = round((best - t) / 360.0)
        travel = best - c
        final.append(best)
        within = 'yes' if abs(travel) <= mjump else 'NO'
        print(f'  joint_{i + 1:<2} {t:>10.3f} {c:>10.3f} {best:>10.3f} '
              f'{turns:>+7d} {travel:>+9.3f} {within:>7}')
    print()
    print(f'  the driver sends: {[round(v, 3) for v in final]}')
    print()

    print('  Which is what robot.move_j() is called with:')
    print(f'      robot.move_j({", ".join(f"{v:.3f}" for v in final)})')
    print()

    print('=' * 74)
    print('STEP 4 -- where the disagreement sits')
    print('=' * 74)
    print('  Compare the two ways of getting an angle for j3:')
    s3 = sign[2]
    print(f'    raw readback               = {current_raw[2]:.3f}')
    print(f'    raw -> URDF                = '
          f'{sign[2] * (current_raw[2] - joff[2]):.3f}   '
          f'(sign*(raw-joint_offset))')
    print(f'    the target chain implies   = '
          f'{current_raw[2] / s3 + joff[2]:.3f}   (raw/sign+joint_offset)')
    print('  Those differ, and for j3 the difference is about a half turn,')
    print('  which is exactly the interval where the rounding picks the wrong')
    print('  whole turn. Settling this is the real fix; normalisation can only')
    print('  paper over it.')
    print()
    print('  Run tools/probe_range.py to measure the range the device accepts')
    print('  per joint; the target must be folded into that range, not into')
    print('  [0,360) and not to the nearest whole turn.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
