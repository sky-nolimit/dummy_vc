#!/usr/bin/env python3
"""Work out j3's offset correction, since folding alone cannot fix it.

The sweep gave the accepted interval for joint_3 in move_j space:

    accepted: [45, 180]        (and nothing outside)

The driver, for the pose we call "URDF j3 = 0", computes move_j j3 = -1.474.
That is refused. But so is every whole-turn shift of it:

    -1.474 - 360 = -361.474    outside
    -1.474         =  -1.474    outside
    -1.474 + 360 =  358.526    outside
    -1.474 + 720 =  718.526    outside

So this is NOT a whole-turn problem. The value the driver produces is in a
different place in the joint's travel than the value the device accepts. Since
the accepted interval is 90 degrees away from where the driver is aiming, at
least one of joint_sign[3] / joint_offset_deg[3] / move_j_offset_deg[3] has to
change -- folding can only remap by whole turns.

The correct target is not in doubt, because it is pinned twice over:
  * commanded -1.474 to the device directly and the arm stayed put;
  * commanded 90 (and 91.474) and the arm went to the 7 pose.
So the target for "URDF j3 = 0" must be ~90.

This script solves for the offset that produces that, and checks the whole
joint set.

Usage:
    python tools/solve_j3_offset.py
"""

SIGN = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
JOFF = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
MJOFF = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

# measured by probe_range (move_j space, degrees)
RANGES = [(-165.0, 165.0), (-75.0, 90.0), (45.0, 180.0),
          (-180.0, 165.0), (-90.0, 90.0), (-180.0, 180.0)]

# "URDF all zeros" is the 7 pose, and the device reaches it at these move_j
# values -- verified by homing() and by the accepted sweep.
MOVEJ_7POSE = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]


def target_with(sign, joff, mjoff, urdf_deg):
    """The driver's chain: urdf -> angle -> move_j."""
    out = []
    for i in range(6):
        s = sign[i] if abs(sign[i]) > 1e-12 else 1.0
        angle = urdf_deg[i] / s + joff[i]
        out.append(angle + mjoff[i])
    return out


def main():
    print('=' * 74)
    print('1. WHAT THE DRIVER PRODUCES FOR "URDF = 0" (the 7 pose)')
    print('=' * 74)
    cur = target_with(SIGN, JOFF, MJOFF, [0.0] * 6)
    print(f'  computed move_j = {[round(v, 3) for v in cur]}')
    print(f'  device accepts  = [45, 180] for joint_3')
    print(f'  joint_3 = {cur[2]:.3f} is OUTSIDE -> the whole command is dropped')
    print()

    print('=' * 74)
    print('2. WHAT IT MUST BE, FOR EACH JOINT')
    print('=' * 74)
    print(f'  {"joint":<8} {"computed":>10} {"accepted":>20} {"ok":>5} '
          f'{"needed shift":>14}')
    for i in range(6):
        lo, hi = RANGES[i]
        ok = lo - 1e-6 <= cur[i] <= hi + 1e-6
        # the smallest whole-turn shift that would land inside
        shift = None
        for k in range(-3, 4):
            v = cur[i] + 360.0 * k
            if lo - 1e-6 <= v <= hi + 1e-6:
                shift = 360.0 * k
                break
        print(f'  joint_{i + 1:<2} {cur[i]:>10.3f} [{lo:>7.1f},{hi:>7.1f}] '
              f'{"yes" if ok else "NO":>5} '
              f'{("%+d" % shift) if shift is not None else "none fits":>14}')
    print()

    print('=' * 74)
    print('3. SO WHAT IS WRONG WITH joint_3')
    print('=' * 74)
    want = MOVEJ_7POSE[2]
    print(f'  wanted move_j = {want:.1f}, got {cur[2]:.3f}')
    print(f'  difference    = {cur[2] - want:+.3f} deg')
    print()
    print('  The chain is:  angle = urdf/sign + joint_offset_deg')
    print('                 move_j = angle + move_j_offset_deg')
    print(f'  For joint_3:   angle = 0/{SIGN[2]:.0f} + ({JOFF[2]:.3f}) = '
          f'{0 / SIGN[2] + JOFF[2]:.3f}')
    print(f'                 move_j = {0 / SIGN[2] + JOFF[2]:.3f} + '
          f'{MJOFF[2]:.1f} = {cur[2]:.3f}')
    print()
    print(f'  For move_j to come out at {want:.1f}, angle must be '
          f'{want - MJOFF[2]:.1f}.')
    print(f'  angle = urdf/sign + offset, and urdf = 0, so offset must be '
          f'{want - MJOFF[2]:+.1f}.')
    print(f'  The config has {JOFF[2]:+.3f}.')
    print()

    print('=' * 74)
    print('4. VERIFY THE CORRECTED OFFSET AGAINST EVERYTHING WE KNOW')
    print('=' * 74)
    fixed_joff = list(JOFF)
    fixed_joff[2] = want - MJOFF[2]
    print(f'  corrected joint_offset_deg = '
          f'{[round(v, 3) for v in fixed_joff]}')
    print()
    fixed = target_with(SIGN, fixed_joff, MJOFF, [0.0] * 6)
    print(f'  URDF 0  -> move_j = {[round(v, 3) for v in fixed]}')
    print(f'  all inside their intervals: '
          f'{all(RANGES[i][0] - 1e-6 <= fixed[i] <= RANGES[i][1] + 1e-6 for i in range(6))}')
    print()
    # a physical rotation of j3 by +30 deg changes URDF by -30 (sign -1)
    rot = [0.0] * 6
    rot[2] = -30.0
    r = target_with(SIGN, fixed_joff, MJOFF, rot)
    print(f'  rotate j3 by +30 deg physical (URDF {rot[2]:+.0f}) -> '
          f'move_j j3 = {r[2]:.3f}')
    print(f'    inside [45, 180]: '
          f'{45 - 1e-6 <= r[2] <= 180 + 1e-6}')
    print(f'    travel from the 7 pose: {r[2] - fixed[2]:+.3f} deg')
    print()
    print('  Note the sign convention: a POSITIVE physical rotation makes URDF')
    print('  go negative here, because joint_sign[3] is -1. If the arm turns')
    print('  the wrong way in practice, that is the sign to flip -- but the')
    print('  offset above is what makes the target land inside the accepted')
    print('  interval, and that part is not a matter of opinion.')

    print()
    print('=' * 74)
    print('5. DOES THIS BREAK THE OTHER JOINTS?')
    print('=' * 74)
    for i in range(6):
        if i == 2:
            continue
        lo, hi = RANGES[i]
        ok = lo - 1e-6 <= fixed[i] <= hi + 1e-6
        print(f'  joint_{i + 1}: move_j {fixed[i]:>9.3f} in '
              f'[{lo:>7.1f},{hi:>7.1f}]  {"ok" if ok else "OUT OF RANGE"}')
    print()
    print('  Only joint_3 changed, so nothing else moves.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
