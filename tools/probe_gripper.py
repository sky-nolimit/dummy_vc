#!/usr/bin/env python3
"""Calibrate the gripper against the device, in stages, read-only first.

The plan (docs/01 §4 stages 0.4-0.6) needs three things measured:

    0.4  the angle range, and what angle 0 means (closed or open)
    0.5  the angle -> finger opening mapping, so /joint_states can publish the
         two prismatic finger joints and so a target width can be turned into a
         command
    0.6  the current limit that grips without slipping and without crushing

None of it can be derived: the URDF says the fingers travel 0..0.023 m, while
the device interface takes `hand.angle`, and no published relation connects
them. It has to be measured.

**This script does nothing on its own.** Stages are selected explicitly and
each one prints what it will do before doing it. Stage 0 is read-only.

SAFETY, from the plan's risk list (R9)
--------------------------------------
`hand.set_current_limit()` set too high can crush an object, throw it, or press
a finger against its stop -- all of which can damage the mechanism. So:

  * stage 0 touches nothing; run it first and read the output
  * stage 2 starts at `--current-start 0.1` and steps up; stop as soon as the
    gripper grips the way you want, and do not keep going "to see what happens"
  * test the current limit on something fragile (a paper cup, an egg) and on
    nothing at all before trusting it
  * the fingers move fast; keep them clear

Usage
    pkill -f dummy_controller
    python3 tools/probe_gripper.py --stage 0
    python3 tools/probe_gripper.py --stage 1 --angle 0,10,20,30,45,60,90
    python3 tools/probe_gripper.py --stage 2 --current-start 0.1 --current-step 0.1
"""
import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

EPS = 0.5          # degrees of change that still counts as "moving"
QUIET_S = 0.8


def with_timeout(fn, timeout, label):
    """Run `fn` on a daemon thread; give up after `timeout` seconds.

    Needed because a fibre call can block forever: an earlier version of the
    driver hung here and held the link, which starved every later command. A
    daemon thread plus a deadline keeps the probe usable and reports the block
    instead of hanging.
    """
    box = {}

    def run():
        try:
            box['value'] = fn()
        except Exception as exc:  # noqa: BLE001
            box['error'] = exc

    th = threading.Thread(target=run, daemon=True)
    th.start()
    th.join(timeout)
    if th.is_alive():
        print(f'    BLOCKED: {label} did not return within {timeout}s '
              '(the call is still pending; treat the link as held)')
        return None, True
    if 'error' in box:
        print(f'    RAISED: {label}: {box["error"]}')
        return None, False
    return box.get('value'), False


def read_angle(hand, timeout=5.0):
    val, blocked = with_timeout(lambda: float(getattr(hand, 'angle', float('nan'))),
                                timeout, 'hand.angle')
    return val


def wait_still(hand, timeout=15.0):
    """Poll hand.angle until it stops changing."""
    t0 = time.monotonic()
    last = read_angle(hand)
    if last is None:
        return None, 0.0, True
    stable = time.monotonic()
    while time.monotonic() - t0 < timeout:
        time.sleep(0.2)
        now = read_angle(hand)
        if now is None:
            continue
        if abs(now - last) > EPS:
            stable = time.monotonic()
        elif time.monotonic() - stable > QUIET_S:
            return now, time.monotonic() - t0, False
        last = now
    return last, time.monotonic() - t0, True


def stage0(hand, robot, args):
    """Read-only. Establish what can be observed BEFORE commanding anything.

    This matters more than it looks. The plan's interface dump lists only
    `hand.set_angle`, `hand.set_enable` and `hand.set_current_limit` -- no
    readback. The driver reads `hand.angle` inside a getattr(..., 0.0) fallback,
    which would silently report 0 forever if the attribute does not exist, and
    the finger positions published to /joint_states would be wrong with no
    warning anywhere.

    So the first question is whether the gripper can be OBSERVED at all. If it
    cannot, calibration has to be done by eye against a ruler, and that changes
    the plan for stages 1 and 2 -- better to find out now, touching nothing.
    """
    print('  stage 0: read only. Nothing is commanded.')
    print()

    print('  gripper attributes visible on the device:')
    names = [n for n in dir(hand) if not n.startswith('_')]
    setters = [n for n in names if n.startswith('set_')]
    others = [n for n in names if not n.startswith('set_')]
    print(f'    set_* methods ({len(setters)}): {setters}')
    print(f'    everything else ({len(others)}): {others}')
    print()

    has_angle = 'angle' in names
    print(f'  hand.angle exists: {has_angle}')
    if has_angle:
        val = read_angle(hand)
        print(f'    value now = {val}')
        # read twice a moment apart: a live value will be steady when idle,
        # and a stub returning 0.0 forever is worth knowing about
        time.sleep(0.5)
        val2 = read_angle(hand)
        print(f'    value 0.5 s later = {val2}')
        if val is not None and val2 is not None:
            t0 = time.monotonic()
            seen = []
            while time.monotonic() - t0 < 2.0:
                v = read_angle(hand)
                if v is not None:
                    seen.append(v)
                time.sleep(0.2)
            if seen and max(seen) - min(seen) < 1e-6:
                print(f'    the reading never changes over 2 s ({seen[0]})')
                print('      that is expected while idle, but it also means a')
                print('      hardcoded 0.0 would look identical -- stage 1 is')
                print('      what proves it is live')
    else:
        print('    NO READBACK. The gripper is open loop: you can command it')
        print('    but not observe it. Calibration then has to be done by eye')
        print('    (hold a ruler against the fingers), and /joint_states cannot')
        print('    report finger positions at all -- the two finger TF frames')
        print('    will stay missing in RViz.')
        print('    The driver currently reads it inside getattr(..., 0.0), so')
        print('    it would silently publish 0 forever.')
    print()

    print('  device temperature / voltage (a rough sign of whether the')
    print('  gripper is powered; neither is a torque reading, so neither can')
    print('  tell you the gripping force):')
    for name in ('get_temperature', 'get_voltage'):
        fn = getattr(robot, name, None)
        if fn is None:
            print(f'    robot.{name}: not present')
            continue
        v, blocked = with_timeout(fn, 4.0, f'robot.{name}()')
        if v is not None:
            print(f'    robot.{name}() = {v}')
    print()
    print('  joint_1..6 temperature, if the joints expose it:')
    found_any = False
    for i in range(1, 7):
        probe = getattr(robot, f'joint_{i}', None)
        fn = getattr(probe, 'get_temperature', None) if probe is not None else None
        if fn is None:
            continue
        found_any = True
        t, blocked = with_timeout(fn, 4.0, f'joint_{i}.get_temperature()')
        if t is not None:
            print(f'    joint_{i}: {t}')
    if not found_any:
        print('    none of joint_1..6 expose get_temperature')
    print()
    print('  Nothing was commanded. Stage 1 moves the fingers; run it only')
    print('  when you are ready to watch them.')
    return 0


def stage1(hand, args):
    angles = [float(v) for v in args.angle.split(',')]
    print(f'  stage 1: sweep hand.set_angle through {angles}')
    print('  The fingers will move through their whole travel.')
    print()
    print(f'  {"commanded":>10} {"readback":>10} {"settle s":>9}  note')
    for a in angles:
        val, blocked = with_timeout(lambda a=a: hand.set_angle(a), 8.0,
                                    f'set_angle({a})')
        if blocked:
            print(f'  {a:>10.1f} {"-":>10} {"-":>9}  command blocked; stopping')
            return 1
        got, elapsed, trunc = wait_still(hand, args.timeout)
        note = '  <-- still moving at timeout' if trunc else ''
        print(f'  {a:>10.1f} {("" if got is None else f"{got:>10.3f}")} '
              f'{elapsed:>9.1f}{note}')
    print()
    print('  Read the table like this:')
    print('    * the smallest and largest readback bound the travel')
    print('    * which END is open: the readback where the fingers are apart')
    print('    * whether the readback moves monotonically with the command,')
    print('      which tells you the SIGN of the mapping')
    print()
    print('  Lock in the two ends and the URDF prismatic mapping follows.')
    return 0


def stage2(hand, args):
    print('  stage 2: scan hand.set_current_limit, watching for grip.')
    print(f'  starting at {args.current_start}, step {args.current_step}, '
          f'up to {args.current_max}')
    print()
    print('  Put the object you want to grip between the fingers first.')
    print('  Stop the scan (Ctrl-C) as soon as the grip looks right; do not')
    print('  keep raising the current just to see what happens.')
    print()
    # park open so the object can be placed, then close on it each round
    a_open = args.open_angle
    a_close = args.close_angle
    print(f'  each round: set_angle({a_close}) at the given limit, then read')
    print()
    print(f'  {"limit":>8} {"angle closed at":>16} {"settle s":>9}  note')
    cur = args.current_start
    rows = []
    while cur <= args.current_max + 1e-9:
        val, blocked = with_timeout(lambda c=cur: hand.set_current_limit(c),
                                    8.0, f'set_current_limit({cur})')
        if blocked:
            print(f'  {cur:>8.2f}  command blocked; stopping')
            return 1
        # open, then close, so each round starts from the same place
        with_timeout(lambda: hand.set_angle(a_open), 8.0, 'set_angle(open)')
        wait_still(hand, args.timeout)
        with_timeout(lambda a=a_close: hand.set_angle(a), 8.0,
                     f'set_angle({a_close})')
        got, elapsed, trunc = wait_still(hand, args.timeout)
        note = ''
        if trunc:
            note = ' still moving at timeout'
        elif got is not None and abs(got - a_close) > 2.0:
            note = ' STOPPED SHORT: the grip stalled the fingers'
        rows.append((cur, got, note))
        print(f'  {cur:>8.2f} {("" if got is None else f"{got:>16.3f}")} '
              f'{elapsed:>9.1f}{note}')
        cur = round(cur + args.current_step, 6)
    print()
    print('  The lowest limit whose readback stops short of the commanded')
    print('  angle is the one that grips. That value, not a guess, is what')
    print('  grip_current_limit should be set to.')
    print()
    print('  Leave the gripper open and the limit low before you walk away.')
    with_timeout(lambda: hand.set_angle(a_open), 8.0, 'set_angle(open)')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stage', type=int, required=True, choices=(0, 1, 2),
                    help='0 read-only, 1 angle sweep, 2 current scan')
    ap.add_argument('--angle', default='0,10,20,30,45,60,90',
                    help='stage 1: commanded angles')
    ap.add_argument('--open-angle', type=float, default=90.0,
                    help='stage 2: the angle that opens the fingers')
    ap.add_argument('--close-angle', type=float, default=0.0,
                    help='stage 2: the angle to close to')
    ap.add_argument('--current-start', type=float, default=0.1)
    ap.add_argument('--current-step', type=float, default=0.1)
    ap.add_argument('--current-max', type=float, default=1.0)
    ap.add_argument('--timeout', type=float, default=15.0)
    ap.add_argument('--yes', action='store_true',
                    help='skip the confirmation prompt')
    args = ap.parse_args()

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except ImportError as exc:
        print(f'FATAL: cannot import dummy_cli_tool ({exc})')
        return 2

    if args.stage > 0 and not args.yes:
        print(f'About to run stage {args.stage}, which MOVES THE FINGERS.')
        print('Keep them clear, and have the power switch within reach.')
        try:
            reply = input('Type "yes" to continue: ').strip().lower()
        except EOFError:
            reply = ''
        if reply != 'yes':
            print('aborted')
            return 1

    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first -- only one')
        print('       process can hold the fibre link.')
        return 1
    robot = dev.robot
    hand = robot.hand

    print('=== probe_gripper ===')
    print()
    print('  the gripper needs its OWN enable; robot.set_enable() does not')
    print('  cover it (docs/01 §2, blocking issue 6)')
    val, blocked = with_timeout(lambda: hand.set_enable(True), 8.0,
                                'hand.set_enable(True)')
    if blocked:
        print('  FATAL: enabling the gripper blocked. Stopping before anything')
        print('         else touches the link.')
        return 1
    print('  gripper enabled')
    print()

    if args.stage == 0:
        return stage0(hand, robot, args)
    if args.stage == 1:
        return stage1(hand, args)
    return stage2(hand, args)


if __name__ == '__main__':
    sys.exit(main())
