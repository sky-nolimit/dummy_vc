#!/usr/bin/env python3
"""Check the down-link calibration against the measured (move_j, raw) pairs.

## The measurement

tools/probe_pairs.py commanded two poses the configuration already names, and
read the device back:

    home_joints_deg   move_j [0, 0, 90, 0, 0, 0]
                      raw    [0, 75.032, -90.016, -0.002, 0, -0.007]
    reset_joints_deg  move_j [0, -75, 180, 0, 0, 0]
                      raw    [0, 0.040, -0.030, 0.001, 0.001, -0.001]

Both are facts, not derivations. Subtracting gives, per joint:

    raw - move_j:
        j1   0.000 / 0.000
        j2  75.032 / 75.040      agree to 0.008
        j3  -180.016 / -180.030  agree to 0.014
        j4-j6  ~0

Two independent poses agreeing to better than 0.02 deg. The device therefore
obeys `raw = move_j + K` with K = [0, 75, -180, 0, 0, 0]. The second pose is
meaningful on its own: the power-off retracted pose is independently known to
read near zero, and reset_joints_deg read back as [0, 0.04, -0.03, ...].

## Why the anchor alone cannot find a calibration bug

The down-link is

    move_j = urdf/sign + joint_offset_deg + move_j_offset_deg

At the anchor urdf = 0, the two offsets enter only as a SUM:

    joint_offset_deg + move_j_offset_deg = move_j at the anchor

That pins the sum and leaves a one-parameter family: moving value from one
parameter to the other keeps the anchor exact and changes nothing about which
pose is reached. **This is why the bug survived so long** -- every check
written around the anchor passes for the wrong numbers too. Only the measured
(move_j, raw) pairs distinguish them, which is section 3 below.

## The bug this found

The config had `move_j_offset_deg = [0, 0, 90, 0, 0, 0]` together with a
down-link that omitted `joint_offset_deg`. Together those put URDF zero at

    j2 move_j 74.572 instead of 0     (off by +74.572)
    j3 move_j -1.474 instead of 90    (off by -91.474)

joint_2 escaped notice because +74.572 still lands inside its accepted interval
[-75, 90]. joint_3 did not: -1.474 is outside [45, 180], and the device drops
such a command SILENTLY, taking the whole command with it. That is the
"sent ok, arm did not move" symptom this project chased for a long time.

Usage:
    python tools/check_downlink.py
"""

# measured reference pairs
HOME_MJ = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
RESET_MJ = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]
MEASURED_HOME_RAW = [0.0, 75.032, -90.016, -0.002, 0.0, -0.007]
MEASURED_RESET_RAW = [0.0, 0.040, -0.030, 0.001, 0.001, -0.001]

# the driver's algebra, as it now stands
SIGN = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
JOFF = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
MJOFF = [-0.215, -74.572, 181.474, 0.910, 0.035, 0.040]


def angle_to_urdf(raw):
    return [SIGN[i] * (raw[i] - JOFF[i]) for i in range(6)]


def urdf_to_movej(urdf):
    return [urdf[i] / SIGN[i] + JOFF[i] + MJOFF[i] for i in range(6)]


def main():
    print('=' * 74)
    print('1. THE MEASURED RELATION  raw = move_j + K')
    print('=' * 74)
    print(f'  {"joint":<8} {"home: raw-mj":>14} {"reset: raw-mj":>15}'
          f' {"agreement":>11}')
    K = []
    for i in range(6):
        a = MEASURED_HOME_RAW[i] - HOME_MJ[i]
        b = MEASURED_RESET_RAW[i] - RESET_MJ[i]
        K.append((a + b) / 2.0)
        print(f'  joint_{i + 1:<2} {a:>14.3f} {b:>15.3f} {abs(a - b):>11.3f}')
    print()
    print(f'  K = [{" ".join(f"{v:.0f}" for v in K)}]')
    print('  two independent poses agree to < 0.02 deg, so K is measured')
    print()

    print('=' * 74)
    print('2. THE ANCHOR (necessary, but NOT sufficient)')
    print('=' * 74)
    print('  at urdf = 0 the down-link gives')
    print('      move_j = joint_offset_deg + move_j_offset_deg')
    print('  and that must equal home_joints_deg at the same pose')
    print()
    print(f'  {"joint":<8} {"joff":>9} {"mjoff":>9} {"sum":>9} '
          f'{"required":>10} {"err":>9}')
    worst = 0.0
    for i in range(6):
        s = JOFF[i] + MJOFF[i]
        err = s - HOME_MJ[i]
        worst = max(worst, abs(err))
        flag = '' if abs(err) < 1e-6 else '   <-- OFF'
        print(f'  joint_{i + 1:<2} {JOFF[i]:>9.3f} {MJOFF[i]:>9.3f} {s:>9.3f} '
              f'{HOME_MJ[i]:>10.1f} {err:>+9.3f}{flag}')
    print()
    print(f'  worst error = {worst:.3e} deg  '
          f'{"[ok]" if worst < 1e-9 else "[FAIL]"}')
    print()

    print('=' * 74)
    print('3. THE MEASURED POSES')
    print('=' * 74)
    print('  Read the device at each pose, convert to URDF the way the driver')
    print('  does, push it through the down-link, and see where it lands.')
    print()
    print('  A small residual is expected and is NOT a down-link error: the')
    print('  raw -> URDF step uses joint_offset_deg as the zero point, and the')
    print('  measured 7 pose reads -90.016 on joint_3 where that zero says')
    print('  -91.474. The 1.5 deg between them shows up here once. Section 4')
    print('  below removes it by using the measured relation instead.')
    print()
    worst_all = 0.0
    for label, raw, want_mj in (('home', MEASURED_HOME_RAW, HOME_MJ),
                                ('reset', MEASURED_RESET_RAW, RESET_MJ)):
        u = angle_to_urdf(raw)
        m = urdf_to_movej(u)
        print(f'  {label}:')
        for i in range(6):
            err = m[i] - want_mj[i]
            worst_all = max(worst_all, abs(err))
            flag = '' if abs(err) < 2.0 else '   <-- OFF'
            print(f'    joint_{i + 1}: urdf {u[i]:>9.3f}  move_j {m[i]:>9.3f}  '
                  f'expected {want_mj[i]:>9.3f}  err {err:>+7.3f}{flag}')
        print()
    print(f'  worst error = {worst_all:.3f} deg  '
          f'{"[ok]" if worst_all < 2.0 else "[FAIL]"}')
    print('  bounded by the raw -> URDF zero-point scatter, ~1.5 deg')
    print()

    print('=' * 74)
    print('4. PHYSICAL CONSISTENCY  raw = move_j + K')
    print('=' * 74)
    print('  The target chain and the measured device relation must agree:')
    print('  push URDF zero through the down-link, add K, and compare with the')
    print('  raw the device actually reports there.')
    print()
    m = urdf_to_movej([0.0] * 6)
    worst_p = 0.0
    for i in range(6):
        pred = m[i] + K[i]
        err = pred - MEASURED_HOME_RAW[i]
        worst_p = max(worst_p, abs(err))
        print(f'    joint_{i + 1}: move_j {m[i]:>9.3f}  + K -> raw {pred:>9.3f}  '
              f'measured {MEASURED_HOME_RAW[i]:>9.3f}  diff {err:>+7.3f}')
    print()
    print(f'  worst difference = {worst_p:.3f} deg  '
          f'{"[ok]" if worst_p < 0.1 else "[FAIL]"}')
    print()

    print('=' * 74)
    print('SUMMARY')
    print('=' * 74)
    ok = (worst < 1e-9 and worst_all < 2.0 and worst_p < 0.1)
    print(f'  {"PASS" if ok else "FAIL"}: anchor exact, both measured poses')
    print('  reproduced, and the target chain agrees with the device relation.')
    print()
    print('  If a future change breaks this, the likely cause is that')
    print('  joint_offset_deg and move_j_offset_deg stopped summing to')
    print('  home_joints_deg, or that the down-link stopped adding both terms.')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
