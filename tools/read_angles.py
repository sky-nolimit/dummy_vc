#!/usr/bin/env python3
"""Read the six raw joint angles -- what a zero-offset calibration needs.

Why this is a tool and not a one-liner
--------------------------------------
Every number in `dummy_controller.yaml` starts life as a raw readback angle, and
the only way to get one is to ask the device. Done by hand that means writing a
fresh fibre script each time, and two mistakes are easy to make:

  * forgetting that `robot.set_enable` is what applies torque, so the "free" pose
    never becomes free -- the arm reports the pose and the pose does not change;
  * reporting a raw angle without saying what the current config turns it into,
    so a pose that was posed BY HAND cannot be compared with what RViz believes.

This tool reads the angles, shows what the config currently makes of them, and
computes the `urdf_offset_deg` that would make THIS pose read as all zeros.

Usage
-----
    python3 tools/read_angles.py --limp        # cut torque, pose by hand, read
    python3 tools/read_angles.py --spin-up     # powered pose, read
    python3 tools/read_angles.py --repeat 5    # is the readback stable?
    python3 tools/read_angles.py --check-home  # still at the calibrated home?
    python3 tools/read_angles.py --config-only # no device; just print the config

`--check-home` compares the live readback with `home_reference_deg` from the
config and exits 3 when a joint is further than `--home-tol` (1.0 deg) from it.
That turns "somebody nudged j6 before the driver came up" and "the device frame
shifted after a power cycle" into a number, instead of a slightly crooked arm
noticed halfway through a grasp.

`--limp` disables the axes and leaves them disabled: the controller itself stays
powered (USB), so the encoders keep reporting while the arm can be pushed into
place. That is the hand-calibration workflow.

    WARNING: a limp arm does not hold itself up. Support it before --limp.

The offset arithmetic
---------------------
    readback   urdf = sign * (angle - joint_offset_deg) + urdf_offset_deg
    command    move_j = (urdf - urdf_offset_deg) / sign + movej_urdf_zero_deg

so a pose whose URDF angles are all zero (the ideal home) needs

    urdf_offset_deg = sign * (joint_offset_deg - angle)

per joint. `urdf_offset_deg` is applied to BOTH chains (commit 05ce916), so one
number fixes the readback and the command at once.
"""
import argparse
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT_CONFIG = os.path.join(
    REPO, 'dummy_vc_ws', 'src', 'dummy_controller', 'config',
    'dummy_controller.yaml')

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Degrees. Below this two reads are "the same number"; the arm has shown 0.02 deg
# between consecutive reads, so this leaves an order of magnitude of headroom.
STABLE_DEG = 0.05


# ----------------------------------------------------------------------
# config arithmetic (no ROS, no device -- this half runs anywhere)
# ----------------------------------------------------------------------


def parse_flow_list(text, key):
    """Read one `key: [a, b, ...]` line. None if the key is absent.

    Deliberately not yaml_min: `dummy_controller.yaml` carries long CJK comment
    blocks that a full reader has choked on before (see the note in
    selftest_follow_joint_trajectory.py), and these three parameters are plain
    inline flow lists on a single line.
    """
    m = re.search(rf'^\s*{re.escape(key)}\s*:\s*\[([^\]]*)\]', text, re.M)
    if m is None:
        return None
    out = []
    for chunk in m.group(1).split(','):
        chunk = chunk.split('#')[0].strip()
        if chunk:
            out.append(float(chunk))
    return out


def read_config_lists(path=DEFAULT_CONFIG):
    """(sign, joint_offset_deg, urdf_offset_deg), or (None, None, None)."""
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    sign = parse_flow_list(text, 'joint_sign')
    joff = parse_flow_list(text, 'joint_offset_deg')
    off = parse_flow_list(text, 'urdf_offset_deg')
    if sign is None or joff is None:
        return None, None, None
    if off is None:
        # The key is in the shipped config; treat a missing one as zeros rather
        # than failing, so this still runs against an older checkout.
        off = [0.0] * len(joff)
    for name, vals in (('joint_sign', sign), ('joint_offset_deg', joff),
                       ('urdf_offset_deg', off)):
        if len(vals) != len(joff):
            raise ValueError(f'{name} has {len(vals)} entries, '
                             f'expected {len(joff)}')
    return sign, joff, off


def urdf_angles(raw, sign, joff, off):
    """The URDF angles the config turns these raw angles into."""
    return [s * (a - o) + f for a, o, s, f in zip(raw, joff, sign, off)]


def offsets_for_home(raw, sign, joff):
    """The urdf_offset_deg that makes `raw` (the ideal home) read all zeros."""
    return [s * (o - a) for a, s, o in zip(raw, sign, joff)]


def compare_to_reference(raw, reference):
    """Per-joint (raw - reference), degrees. Positive = past the reference."""
    return [a - b for a, b in zip(raw, reference)]


def read_home_reference(path=DEFAULT_CONFIG):
    """The raw angles recorded at the calibrated home, or None."""
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    ref = parse_flow_list(text, 'home_reference_deg')
    if ref is None or len(ref) != len(JR):
        return None
    return ref


def spread(readings):
    """Per-joint (max - min) across a list of reading lists, in degrees."""
    return [max(col) - min(col) for col in zip(*readings)]


# ----------------------------------------------------------------------
# the device half
# ----------------------------------------------------------------------


def read_once(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Read the six raw joint angles and the offset they imply.')
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--limp', action='store_true',
                      help='disable the axes first (pose the arm by hand)')
    mode.add_argument('--spin-up', dest='spin_up', action='store_true',
                      help='enable the axes first (read a powered pose)')
    ap.add_argument('--repeat', type=int, default=1, metavar='N',
                    help='read N times and report the spread (default 1)')
    ap.add_argument('--interval', type=float, default=0.5, metavar='S',
                    help='seconds between reads (default 0.5)')
    ap.add_argument('--config', default=DEFAULT_CONFIG,
                    help='controller config to compare against')
    ap.add_argument('--config-only', dest='config_only', action='store_true',
                    help='print the config tables and exit (no device needed)')
    ap.add_argument('--check-home', dest='check_home', action='store_true',
                    help='exit 3 when a joint is off the recorded home')
    ap.add_argument('--home-tol', dest='home_tol', type=float, default=1.0,
                    metavar='DEG',
                    help='how far from the recorded home counts as "not at '
                         'home" (default 1.0 deg)')
    ap.add_argument('--timeout', type=float, default=15.0,
                    help='device discovery timeout (default 15 s)')
    args = ap.parse_args(argv)

    sign, joff, off = read_config_lists(args.config)

    if args.config_only:
        if sign is None:
            print(f'FATAL: {args.config} has no joint_sign/joint_offset_deg')
            return 2
        print(f'config: {args.config}')
        print(f'  joint_sign        {sign}')
        print(f'  joint_offset_deg  {joff}')
        print(f'  urdf_offset_deg   {off}')
        ref = read_home_reference(args.config)
        print(f'  home_reference_deg {ref if ref else "(absent)"}')
        return 0

    sys.path.insert(0, HERE)
    from device_session import connect, power_on  # noqa: E402

    robot, _hand = connect(timeout=args.timeout)
    if robot is None:
        return 2

    if args.spin_up:
        if not power_on(robot):
            print('FATAL: the enable call did not return; not reading a pose '
                  'the arm never reached.')
            return 2
        time.sleep(0.5)
    elif args.limp:
        print('disabling the axes -- the arm will not hold itself up; support '
              'it before pushing the joints', flush=True)
        robot.set_enable(False)
        time.sleep(0.5)
    else:
        print('reading with the axes in whatever state they are already in; '
              'use --limp or --spin-up to be explicit', flush=True)

    n = max(1, args.repeat)
    readings = []
    for i in range(n):
        readings.append(read_once(robot))
        if i + 1 < n:
            time.sleep(max(0.0, args.interval))

    raw = readings[-1]
    model = None
    if sign is not None:
        model = urdf_angles(raw, sign, joff, off)

    print()
    print('  joint      raw(deg)   model now(deg)')
    for i, name in enumerate(JR):
        line = f'  {name:<9} {raw[i]:+9.3f}'
        if model is not None:
            line += f'   {model[i]:+9.3f}'
        print(line)

    if n > 1:
        sp = spread(readings)
        worst = max(range(len(sp)), key=lambda k: sp[k])
        print()
        print(f'  {n} reads, largest spread {sp[worst]:.3f} deg on '
              f'{JR[worst]}')
        if sp[worst] > STABLE_DEG:
            print('  NOTE: the readback is moving; make sure the arm is at '
                  'rest before recording this as a calibration pose.')

    if model is None:
        print()
        print(f'  (no joint_sign/joint_offset_deg in {args.config}; '
              f'skipping the offset arithmetic)')
        return 0

    print()
    print('  If this pose is the ideal home (URDF angles all zero), then '
          'urdf_offset_deg should be:')
    want = offsets_for_home(raw, sign, joff)
    print('    [' + ', '.join(f'{v:.3f}' for v in want) + ']')
    moved = [f'{JR[i]} {w - o:+.3f}' for i, (w, o) in enumerate(zip(want, off))
             if abs(w - o) > STABLE_DEG]
    if moved:
        print('  relative to the value in use: ' + ', '.join(moved))
    else:
        print('  (that is what the config already says)')

    rc = 0
    ref = read_home_reference(args.config)
    if ref is not None:
        d = compare_to_reference(raw, ref)
        worst = max(range(len(d)), key=lambda k: abs(d[k]))
        print()
        print('  distance from the recorded home (raw - reference):')
        print('    ' + ', '.join(f'{JR[i]} {v:+.2f}' for i, v in enumerate(d)))
        if abs(d[worst]) > args.home_tol:
            print(f'  WARNING: {JR[worst]} is {d[worst]:+.2f} deg from the '
                  f'recorded home (tolerance {args.home_tol:.1f}).')
            print('           If the arm was just homed, that means the '
                  'device frame moved or the offsets did not apply; if it was '
                  'posed by hand, it is simply not at home.')
            rc = 3
        else:
            print(f'  all six within {args.home_tol:.1f} deg of the recorded '
                  f'home')

    if args.limp:
        print()
        print('  The axes are still disabled. Run the driver (which enables '
              'them) before expecting the arm to hold a pose.')
    return rc if args.check_home else 0


if __name__ == '__main__':
    sys.exit(main())
