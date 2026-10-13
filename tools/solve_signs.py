#!/usr/bin/env python3
"""Solve joint offsets from the measured reference poses.

**本文件的 sign 结论是错的，保留作记录，不要再用。**

它按「读回与指令的位移同向」推 joint_sign，得到 j2 与 j3 都要 +1 —— 这个
答案被几何折叠校验（tools/fold_signs.py）推翻。那个方法只看到两个姿态之间
位移的**方向**，而方向本身取决于各关节的角度约定，不足以定符号。

正确做法是几何：`tools/fold_signs.py` 看收缩姿态是否真的折叠，
再由 `tools/resolve_signs.py` 加上实测命令做 tie-break。

本文件里 **joint_offset 的推导仍然有效** —— 它只依赖「7 姿态的读回值就是
零点」，与符号无关，所以留着。

Facts (tools/probe_pairs.py), consistent at two poses to 0.01 deg:

    raw = move_j + K,   K = [0, 75, -180, 0, 0, 0]

    home  move_j [0, 0, 90, 0, 0, 0]  ->  raw [0, 75.03, -90.02, 0, 0, 0]
    reset move_j [0, -75, 180, 0, 0, 0] -> raw [0, 0.04, -0.03, 0, 0, 0]

The driver's two chains are:

    readback :  urdf   = sign * (raw - joint_offset_deg)
    target   :  move_j = urdf / sign + joint_offset_deg + move_j_offset_deg

Two requirements fix the parameters:

  R1  home is the 7 pose, so the readback there must be URDF zero:
          sign * (raw_home - joint_offset_deg) = 0   ->  joint_offset = raw_home

  R2  the target for URDF zero must be move_j = home:
          joint_offset_deg + move_j_offset_deg = home_joints_deg

R2 also holds for ANY URDF, because both chains add the same constant, so once
R2 is satisfied the target chain is right everywhere -- provided `sign` is right.
`sign` is what R1 alone cannot pin: it cancels when the readback is zero. It is
pinned by the reset pose instead, where the joints actually move.

Usage:
    python tools/solve_signs.py
"""

HOME_RAW = [-0.0, 75.032, -90.016, -0.002, -0.0, -0.007]
RESET_RAW = [-0.0, 0.04, -0.03, 0.001, 0.001, -0.001]
HOME_CMD = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
RESET_CMD = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]

CONFIG_SIGN = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]


def main():
    print('=' * 74)
    print('1. SOLVE joint_offset FROM R1  (offset = raw at the 7 pose)')
    print('=' * 74)
    joff = list(HOME_RAW)
    print(f'  joint_offset_deg = {[round(v, 3) for v in joff]}')
    print()

    print('=' * 74)
    print('2. SOLVE move_j_offset FROM R2  (so the target for URDF 0 is home)')
    print('=' * 74)
    mjoff = [HOME_CMD[i] - joff[i] for i in range(6)]
    print(f'  move_j_offset_deg = {[round(v, 3) for v in mjoff]}')
    print(f'  check sum == home : '
          f'{[round(joff[i] + mjoff[i], 3) for i in range(6)]}')
    print()

    print('=' * 74)
    print('3. SOLVE sign FROM THE RESET POSE  (the only thing that pins it)')
    print('=' * 74)
    print('  Between home and reset, joint i moves by d_raw (readback) and')
    print('  d_mj (command). The readback must track the command, so the URDF')
    print('  change is sign * d_raw, and the command change is d_mj, and the')
    print('  target chain requires  d_mj = (sign*d_raw)/sign = d_raw.')
    print()
    print(f'  {"joint":<8} {"d_raw":>10} {"d_mj":>10} {"ratio":>9} {"sign":>6}')
    sign = []
    for i in range(6):
        d_raw = RESET_RAW[i] - HOME_RAW[i]
        d_mj = RESET_CMD[i] - HOME_CMD[i]
        if abs(d_raw) < 0.05:
            # The joint does not move between these two poses, so this test
            # cannot see its sign. Fall back to the configured value and say so.
            sign.append(CONFIG_SIGN[i])
            note = 'kept from config (no movement here)'
            ratio = float('nan')
        else:
            s = 1.0 if d_raw * d_mj > 0 else -1.0
            sign.append(s)
            ratio = d_mj / d_raw
            note = ''
        print(f'  joint_{i + 1:<2} {d_raw:>+10.3f} {d_mj:>+10.3f} '
              f'{(f"{ratio:+.4f}" if ratio == ratio else "     n/a"):>9} '
              f'{sign[i]:>+6.0f}  {note}')
    print()
    print(f'  joint_sign = {sign}')
    print()

    print('=' * 74)
    print('4. VERIFY: BOTH CHAINS, BOTH POSES')
    print('=' * 74)

    def r2u(raw):
        return [sign[i] * (raw[i] - joff[i]) for i in range(6)]

    def u2m(urdf):
        return [urdf[i] / sign[i] + joff[i] + mjoff[i] for i in range(6)]

    print('  readback (URDF published to the model):')
    for label, raw, want in (('home ', HOME_RAW, [0.0] * 6),
                             ('reset', RESET_RAW, None)):
        u = r2u(raw)
        print(f'    {label}: {[round(v, 3) for v in u]}')
        if want is not None:
            ok = all(abs(u[i] - want[i]) < 0.05 for i in range(6))
            print(f'           expected all zeros: {"ok" if ok else "WRONG"}')
    print()
    print('  target (what gets sent as move_j):')
    for label, raw, want in (('home ', HOME_RAW, HOME_CMD),
                             ('reset', RESET_RAW, RESET_CMD)):
        m = u2m(r2u(raw))
        ok = all(abs(m[i] - want[i]) < 0.1 for i in range(6))
        print(f'    {label}: {[round(v, 3) for v in m]}')
        print(f'           expected {want}  ->  {"ok" if ok else "MISMATCH"}')
    print()

    print('=' * 74)
    print('5. THE VALUES TO WRITE INTO THE CONFIG')
    print('=' * 74)
    print(f'  joint_sign        : [{", ".join(f"{v:.1f}" for v in sign)}]')
    print(f'  joint_offset_deg  : '
          f'[{", ".join(f"{v:.3f}" for v in joff)}]')
    print(f'  move_j_offset_deg : '
          f'[{", ".join(f"{v:.3f}" for v in mjoff)}]')
    print()
    print('  Joints whose sign CHANGED from the config:')
    changed = [i + 1 for i in range(6) if abs(sign[i] - CONFIG_SIGN[i]) > 1e-9]
    print(f'    {changed if changed else "none"}')
    if changed:
        for i in changed:
            print(f'      joint_{i}: config {CONFIG_SIGN[i - 1]:+.0f} '
                  f'-> {sign[i - 1]:+.0f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
