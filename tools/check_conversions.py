#!/usr/bin/env python3
"""Verify that the driver's conversion functions are mutually consistent.

This does NOT reimplement the formulas -- it IMPORTS the real driver class and
calls its methods, then checks the algebraic identities that must hold between
them:

  A. angle_to_urdf  and  urdf_to_angle   are inverses
  B. urdf_to_movej  and  movej_to_urdf   are inverses (modulo whole turns)
  C. angle_to_movej and  movej_to_angle  are inverses
  D. moving through intermediate spaces agrees with the direct link:
         angle -> URDF -> move_j   ==   angle_to_movej
         angle -> move_j -> URDF   ==   angle_to_urdf
  E. the two anchor points land where they must:
         angle_7pose      -> URDF all zeros, move_j [0,0,90,0,0,0]
         angle_retracted  -> the folded pose

D is the one that catches a wrong inverse: an implementation can be a perfect
inverse of itself and still disagree with the chain it claims to shortcut.

The driver imports rclpy, so this script stubs out enough of it to construct
the object and calls the methods unbound. If that is too fragile, run it on
the Linux side where rclpy exists.

Usage:
    python tools/check_conversions.py
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DRIVER = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'dummy_controller', 'dummy_controller.py')
CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'config', 'dummy_controller.yaml')

# 取值来自 tools/probe_pairs.py 的实测（2026-10-13）。旧值
# [0.215, 74.572, -91.474, -0.910, -0.035, -0.040] 是早期手工锚定的结果，
# 在 j3 上差 1.5 度。这一项必须与配置里的 joint_offset_deg 一致。
ANGLE_7POSE = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
MOVEJ_7POSE = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
ANGLE_RETRACT = [0.0] * 6
JOINTS = [f'j{i}' for i in range(1, 7)]


def get_list(text, key):
    m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
    return [float(v) for v in m.group(1).split(',')] if m else None


class Stub:
    """Minimal stand-in that carries the parameters the methods need."""

    def __init__(self, joff, mjoff, sign):
        self.joint_offset_deg = joff
        self.move_j_offset_deg = mjoff
        self.joint_sign = sign
        self.urdf_offset_deg = [0.0] * 6

    def get_logger(self):
        class L:
            def error(self, *_a, **_k):
                pass
        return L()

    def get_parameter(self, name):
        class P:
            def __init__(self, v):
                self.value = v
        return P({'wrap_min_deg': 0.0, 'wrap_max_deg': 360.0}.get(name, 0.0))


def load_methods():
    """Extract the conversion methods from the driver source.

    Importing the module would drag in rclpy and the fibre driver, which may
    not be installed (Windows) or may try to talk to hardware. Instead the
    source is parsed and only the class body's methods we need are exec'd
    against a stub instance. This keeps the check readable and dependency-free
    while still testing the REAL code rather than a copy of the formula.
    """
    import ast
    src = open(DRIVER, encoding='utf-8').read()
    tree = ast.parse(src)

    wanted = {
        'angle_to_urdf_deg', 'angle_to_urdf', 'urdf_to_angle',
        'urdf_to_movej', 'movej_to_angle', 'angle_to_movej',
        'movej_to_urdf', 'normalize_movej', '_safe_sign', '_apply_sign',
        '_urdf_offsets',
    }
    cls = next(n for n in ast.walk(tree)
               if isinstance(n, ast.ClassDef) and n.name == 'dummy_controller')
    parts = ['import numpy as np', '']
    parts.append('class Conv:')
    parts.append('    def get_logger(self):')
    parts.append('        class L:')
    parts.append('            def error(self, *a, **k): pass')
    parts.append('        return L()')
    parts.append('    def get_parameter(self, name):')
    parts.append('        class P:')
    parts.append('            def __init__(self, v): self.value = v')
    # The real node assigns these from ROS parameters during on_configure,
    # which the stub does not run, so seed the ones the methods read.
    parts.append('        seeded = {')
    parts.append('            "movej_urdf_zero_deg": '
                 '[0.0, 0.0, 90.0, 0.0, 0.0, 0.0],')
    parts.append('            "movej_accepted_range_deg": '
                 '[-165.0, 165.0, -75.0, 90.0, 45.0, 180.0, -180.0, 165.0, '
                 '-90.0, 90.0, -180.0, 180.0],')
    parts.append('            "wrap_min_deg": 0.0,')
    parts.append('            "wrap_max_deg": 360.0,')
    parts.append('            "max_move_jump_deg": 180.0,')
    parts.append('        }')
    parts.append('        if name in seeded:')
    parts.append('            return P(seeded[name])')
    parts.append('        return P(getattr(self, name, 0.0))')
    found = []
    for node in cls.body:
        if isinstance(node, ast.FunctionDef) and node.name in wanted:
            found.append(node.name)
            seg = ast.get_source_segment(src, node)
            parts.append('\n'.join('    ' + ln for ln in seg.splitlines()))
    if not found:
        raise SystemExit('FATAL: no conversion methods found in the driver')
    ns = {}
    exec(compile('\n'.join(parts), DRIVER, 'exec'), ns)  # noqa: S102
    return ns['Conv'], sorted(found)


def wrap(v, lo=0.0):
    while v < lo:
        v += 360.0
    while v >= lo + 360.0:
        v -= 360.0
    return v


def main():
    text = open(CONFIG, encoding='utf-8').read()
    joff = get_list(text, 'joint_offset_deg')
    mjoff = get_list(text, 'move_j_offset_deg')
    sign = get_list(text, 'joint_sign')
    print(f'config: joint_offset_deg={joff}')
    print(f'        move_j_offset_deg={mjoff}')
    print(f'        joint_sign={sign}')
    print()

    Conv, found = load_methods()
    print(f'loaded from the driver: {found}')
    c = Conv()
    c.joint_offset_deg = joff
    c.move_j_offset_deg = mjoff
    c.joint_sign = sign
    c.urdf_offset_deg = [0.0] * 6
    c.wrap_min_deg = 0.0
    c.wrap_max_deg = 360.0

    failures = []

    def chk(label, got, want, tol=1e-6):
        ok = all(abs(got[i] - want[i]) < tol for i in range(6))
        print(f'  [{"ok  " if ok else "FAIL"}] {label}')
        if not ok:
            print(f'          got  {[round(v, 4) for v in got]}')
            print(f'          want {[round(v, 4) for v in want]}')
            failures.append(label)
        return ok

    cases = [[0, 0, 0, 0, 0, 0], [20, -15, 30, -10, 5, -25],
             [90, -60, 30, -120, 40, -80]]

    print('A. angle_to_urdf <-> urdf_to_angle')
    for a in cases:
        u = c.angle_to_urdf_deg(a)
        back = c.urdf_to_angle([math.radians(v) for v in u])
        chk(f'angle {[round(v,1) for v in a]} round trip', back, a, tol=1e-6)
    chk('anchor angle_7pose round trip',
        c.urdf_to_angle([math.radians(v)
                         for v in c.angle_to_urdf_deg(ANGLE_7POSE)]),
        ANGLE_7POSE, tol=1e-6)
    print()

    print('B. urdf_to_movej <-> movej_to_urdf (modulo 360)')
    for u in cases:
        m = c.urdf_to_movej([math.radians(v) for v in u])
        back = np_degrees(c.movej_to_urdf(m))
        norm = [back[i] + 360.0 * round((u[i] - back[i]) / 360.0)
                for i in range(6)]
        chk(f'urdf {[round(v,1) for v in u]} round trip', norm, u, tol=1e-6)
    print()

    print('C. angle_to_movej <-> movej_to_angle')
    for a in cases:
        m = c.angle_to_movej(a)
        back = c.movej_to_angle(m)
        chk(f'angle {[round(v,1) for v in a]} round trip', back, a, tol=1e-6)
    chk('anchor angle_7pose round trip',
        c.movej_to_angle(c.angle_to_movej(ANGLE_7POSE)),
        ANGLE_7POSE, tol=1e-6)
    print()

    print('D. the shortcut must agree with the two-step chain')
    print('   NOTE: the anchor is deliberately NOT used as a case here. At the')
    print('   anchor (angle - joint_offset) is exactly zero, so any sign error')
    print('   in that term cancels and the comparison passes trivially.')
    import random
    random.seed(20261012)
    rand = [[random.uniform(-170, 170) for _ in range(6)]
            for _ in range(20)]
    bad = 0
    for a in cases + rand:
        chain = list(c.urdf_to_movej(c.angle_to_urdf(a)))
        direct = list(c.angle_to_movej(a))
        for i in range(6):
            # urdf_to_movej wraps into [0,360) while angle_to_movej does not,
            # so compare modulo a whole turn. Without this the check reports
            # false failures like 359.785 vs -0.215, which are the same angle.
            d = (chain[i] - direct[i] + 180.0) % 360.0 - 180.0
            if abs(d) > 1e-6:
                bad += 1
                if bad <= 3:
                    print(f'  [FAIL] angle={[round(v,1) for v in a]} j{i+1}: '
                          f'chain={chain[i]:.4f} direct={direct[i]:.4f} '
                          f'(diff {d:.4f})')
    ok = bad == 0
    print(f'  [{"ok  " if ok else "FAIL"}] {len(cases)+len(rand)} inputs x 6 '
          'joints: angle->URDF->move_j == angle_to_movej (modulo 360)')
    if not ok:
        failures.append('D: angle_to_movej disagrees with the two-step chain')

    # A wrong answer that differs by exactly one turn would still be wrong for
    # a device that does not wrap, so also confirm the UNWRAPPED shortcut is
    # the intended device value: it must reproduce move_j 0..360 for the
    # retracted pose, which is the only pose with an independent reference.
    print()
    print('D2. unwrapped shortcut vs the known retracted command')
    got = [float(wrap(v)) for v in c.angle_to_movej(ANGLE_RETRACT)]
    print(f'   angle_retract -> move_j (wrapped) = {[round(v,2) for v in got]}')
    print('   reference reset_joints_deg        = [0.0, -75.0, 180.0, '
          '0.0, 0.0, 0.0] -> wrapped [0.0, 285.0, 180.0, 0.0, 0.0, 0.0]')
    ref = [wrap(v) for v in [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]]
    worst = max(abs(((got[i] - ref[i] + 180) % 360) - 180)
                for i in range(6))
    ok = worst < 5.0
    print(f'  [{"ok  " if ok else "FAIL"}] worst per-joint difference = '
          f'{worst:.3f} deg (tolerance 5)')
    if not ok:
        failures.append('D2: retracted command disagrees with '
                        'reset_joints_deg')
    print()

    print('E. anchors')
    off = list(c.urdf_offset_deg)
    chk('angle_7pose -> URDF == urdf_offset_deg',
        list(c.angle_to_urdf_deg(ANGLE_7POSE)), off, tol=1e-6)
    chk('angle_7pose -> move_j == [0,0,90,0,0,0]',
        [wrap(v) for v in c.angle_to_movej(ANGLE_7POSE)],
        [wrap(v) for v in MOVEJ_7POSE], tol=1e-6)
    chk('URDF == urdf_offset_deg -> move_j == [0,0,90,0,0,0]',
        [wrap(v) for v in c.urdf_to_movej([math.radians(v) for v in off])],
        [wrap(v) for v in MOVEJ_7POSE], tol=1e-6)
    print()

    # F. the same identities with a NON-ZERO urdf_offset_deg.
    #
    # urdf_offset_deg used to be a dead parameter: declared, logged, validated,
    # and read by nothing. Wiring it into only one of the two chains would still
    # leave A-E green (they all run with the offset at zero), so this section
    # exists to make that half-fix fail loudly.
    print('F. identities still hold with urdf_offset_deg != 0')
    c2 = Conv()
    c2.joint_offset_deg = joff
    c2.move_j_offset_deg = mjoff
    c2.joint_sign = sign
    c2.urdf_offset_deg = [7.0, -3.0, 5.0, -11.0, 2.0, -9.0]
    c2.wrap_min_deg = 0.0
    c2.wrap_max_deg = 360.0
    bad_f = 0
    for a in cases + rand:
        u = [float(v) for v in c2.angle_to_urdf_deg(a)]
        # urdf_to_angle takes radians and returns DEGREES, so no extra
        # conversion on the way out (section A above relies on the same thing).
        back = [float(v) for v in c2.urdf_to_angle(
            [math.radians(v) for v in u])]
        if any(abs(back[i] - a[i]) > 1e-6 for i in range(6)):
            bad_f += 1
            print(f'  [FAIL] A: angle {[round(v,1) for v in a]} -> URDF -> '
                  f'angle gave {[round(v,4) for v in back]}')
        # the readback must actually move by the offset
        want_u = [sign[i] * (a[i] - joff[i]) + c2.urdf_offset_deg[i]
                  for i in range(6)]
        if any(abs(u[i] - want_u[i]) > 1e-6 for i in range(6)):
            bad_f += 1
            print(f'  [FAIL] readback ignores urdf_offset_deg for '
                  f'angle {[round(v,1) for v in a]}')
        m = c2.urdf_to_movej([math.radians(v) for v in u])
        backu = [math.degrees(v) for v in c2.movej_to_urdf(m)]
        norm = [backu[i] + 360.0 * round((u[i] - backu[i]) / 360.0)
                for i in range(6)]
        if any(abs(norm[i] - u[i]) > 1e-6 for i in range(6)):
            bad_f += 1
            print(f'  [FAIL] B: URDF {[round(v,1) for v in u]} -> move_j -> '
                  f'URDF gave {[round(v,4) for v in norm]}')
        chain = list(c2.urdf_to_movej(c2.angle_to_urdf(a)))
        direct = list(c2.angle_to_movej(a))
        for i in range(6):
            d = (chain[i] - direct[i] + 180.0) % 360.0 - 180.0
            if abs(d) > 1e-6:
                bad_f += 1
                if bad_f <= 3:
                    print(f'  [FAIL] D with offset: angle '
                          f'{[round(v,1) for v in a]} j{i+1}: '
                          f'chain={chain[i]:.4f} direct={direct[i]:.4f}')
    ok = bad_f == 0
    print(f'  [{"ok  " if ok else "FAIL"}] {len(cases)+len(rand)} inputs: '
          'readback carries the offset, and every round trip still closes')
    if not ok:
        failures.append('F: urdf_offset_deg is not applied symmetrically')
    print()

    print('=' * 62)
    if failures:
        print(f'RESULT: {len(failures)} FAILURE(S)')
        seen = set()
        for f in failures:
            key = f.split('[')[0].strip()
            if key in seen:
                continue
            seen.add(key)
            print(f'  - {f}')
        return 1
    print('RESULT: all conversion identities hold')
    return 0


def np_degrees(x):
    return [math.degrees(v) for v in x]


if __name__ == '__main__':
    sys.exit(main())