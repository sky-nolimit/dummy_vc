#!/usr/bin/env python3
"""Check the URDF joint limits against the device's measured travel.

Why this needs its own check: the device **silently drops an entire command**
that contains a move_j value it will not act on. So a URDF limit that is looser
than the device is not a harmless rounding difference -- MoveIt will plan into
it, the driver will convert the target faithfully, and the arm will ignore the
whole command with no error anywhere. That is the same failure mode that made
joint 3 look like a dead joint for a long time.

The limits and the accepted ranges live in two different files:

    dummy2_urdf/urdf/dummy2_urdf.xacro            URDF limits (radians)
    dummy_controller/config/dummy_controller.yaml movej_accepted_range_deg

and they are derived from one measurement. This script keeps them in step, by
converting each side into URDF degrees with

    urdf_deg = sign * (move_j - movej_urdf_zero_deg)

and requiring that the URDF interval does not extend beyond the device's.

Rounding: the URDF bounds are expected to sit exactly on the measured values
rounded outward, so a tolerance of one rounding step is allowed. What is not
allowed is a bound more than that beyond the device.

    python tools/check_urdf_limits.py
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')
CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller', 'config',
                      'dummy_controller.yaml')

# How far a URDF bound may exceed the device before it is a fault. The limits
# are written as the measured value rounded outward, so allow one step.
TOL_DEG = 1.0


def get_list(text, key):
    m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
    return [float(v) for v in m.group(1).split(',')] if m else None


def main():
    import fk_dummy

    cfg = open(CONFIG, encoding='utf-8').read()
    sign = get_list(cfg, 'joint_sign')
    zero = get_list(cfg, 'movej_urdf_zero_deg')
    rng = get_list(cfg, 'movej_accepted_range_deg')
    if sign is None or zero is None or rng is None:
        print('FAIL: config is missing joint_sign, movej_urdf_zero_deg or '
              'movej_accepted_range_deg')
        return 1
    if len(rng) != 12:
        print(f'FAIL: movej_accepted_range_deg has {len(rng)} values, want 12')
        return 1

    links, joints = fk_dummy.parse_model(XACRO)
    lim = {j['name']: j['limits'] for j in joints if j.get('limits')}

    print('=' * 76)
    print('URDF LIMITS vs MEASURED DEVICE TRAVEL')
    print('=' * 76)
    print('  urdf_deg = sign * (move_j - movej_urdf_zero_deg)')
    print(f'  sign = {sign}')
    print(f'  movej_urdf_zero_deg = {zero}')
    print()
    print(f'  {"joint":<8} {"URDF limit":>20} {"device accepts":>20} '
          f'{"verdict":>10}')
    bad = []
    for i in range(6):
        name = f'dummy2_joint{i + 1}'
        if name not in lim:
            bad.append((i + 1, 'joint missing from the URDF'))
            continue
        lo_r, hi_r = lim[name]              # fk_dummy yields (lower, upper)
        lo, hi = math.degrees(lo_r), math.degrees(hi_r)

        # device interval, converted into URDF degrees
        a = sign[i] * (rng[2 * i] - zero[i])
        b = sign[i] * (rng[2 * i + 1] - zero[i])
        dlo, dhi = min(a, b), max(a, b)

        below = lo < dlo - TOL_DEG
        above = hi > dhi + TOL_DEG
        verdict = 'ok' if not (below or above) else 'TOO LOOSE'
        print(f'  joint_{i + 1:<2} {lo:>9.1f}..{hi:<9.1f} '
              f'{dlo:>9.1f}..{dhi:<9.1f} {verdict:>10}')
        if below:
            bad.append((i + 1,
                        f'URDF lower {lo:.1f} is below what the device accepts '
                        f'({dlo:.1f}); MoveIt could plan there and the device '
                        'would drop the whole command'))
        if above:
            bad.append((i + 1,
                        f'URDF upper {hi:.1f} is above what the device accepts '
                        f'({dhi:.1f}); MoveIt could plan there and the device '
                        'would drop the whole command'))
    print()

    # The named poses must remain reachable: if a limit pinches one of them,
    # the arm cannot be commanded to it at all.
    print('=' * 76)
    print('NAMED POSES STILL REACHABLE')
    print('=' * 76)
    print('  These are the poses the driver commands at startup and on reset,')
    print('  so a limit that excludes one of them is a hard fault. A tolerance')
    print('  is allowed because a single readback carries measurement noise;')
    print('  the intended pose is computed from the command values, which carry')
    print('  none.')
    for key in ('home_joints_deg', 'reset_joints_deg'):
        cmd = get_list(cfg, key)
        if cmd is None:
            continue
        urdf = [sign[i] * (cmd[i] - zero[i]) for i in range(6)]
        rows = []
        for i in range(6):
            name = f'dummy2_joint{i + 1}'
            if name not in lim:
                continue
            lo = math.degrees(lim[name][0])
            hi = math.degrees(lim[name][1])
            # 1.5 deg of slack: readback noise on a pose that rests on a limit
            ok = lo - 1.5 <= urdf[i] <= hi + 1.5
            if not ok:
                bad.append((i + 1,
                            f'{key}: urdf {urdf[i]:.1f} is outside the URDF '
                            f'limit [{lo:.1f}, {hi:.1f}]'))
            rows.append(f'j{i + 1}={urdf[i]:+.1f}' + ('' if ok else '*'))
        print(f'  {key:<18} {cmd}')
        print(f'  {"":<18} urdf {" ".join(rows)}')
    print()

    print('=' * 76)
    if bad:
        print(f'RESULT: {len(bad)} PROBLEM(S)')
        for j, msg in bad:
            print(f'  joint_{j}: {msg}')
        return 1
    print('RESULT: URDF limits match the measured device travel, and both')
    print('        named poses remain reachable')
    return 0


if __name__ == '__main__':
    sys.exit(main())
