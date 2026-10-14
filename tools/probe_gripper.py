#!/usr/bin/env python3
"""Calibrate the gripper, by eye, because the gripper cannot be read.

FIRST-RUN FACTS (tools/probe_gripper.py --stage 0, on hardware)
---------------------------------------------------------------
    hand exposes exactly three methods: set_angle, set_current_limit,
    set_enable -- and NO readable attribute at all. The gripper is open loop.

    hand.set_enable(True) did not return within 8 s on the first attempt. The
    device is left in an unknown state by that, so treat the gripper as
    unconfirmed until the steps below say otherwise.

    robot.get_temperature / get_voltage do not exist. joint_1..6.get_temperature
    returns 0, i.e. it is not implemented.

WHAT THAT MEANS FOR CALIBRATION
-------------------------------
The plan (docs/01 §4) asks for an "angle <-> opening mapping" measured by
reading hand.angle. That is impossible here. The only observables are:

    * what you see the fingers do
    * what you measure with a ruler

So this script does not try to read anything. It commands one value, stops, and
prints the exact question to answer. You answer, it records the answer. That is
the calibration; there is no way around it with this hardware.

HOW TO USE IT
-------------
    pkill -f dummy_controller
    python3 tools/probe_gripper.py --stage 1

It walks through a fixed list of angles and asks what you see. Have the fingers
in view. Do not put anything between them for this stage.

    --start 0 --stop 90 --step 10     to change the sweep
    --dry-run                         print the questions, do not move anything
"""
import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))


def with_timeout(fn, timeout, label):
    """Run `fn` on a daemon thread; give up after `timeout` seconds.

    Every gripper call goes through this. `hand.set_enable(True)` hanging is a
    measured fact, not a hypothetical, so a blocked call must be reported
    rather than allowed to freeze the probe with the fibre link held.
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
        return None, True, f'BLOCKED after {timeout}s'
    if 'error' in box:
        return None, False, f'RAISED {type(box["error"]).__name__}: {box["error"]}'
    return box.get('value'), False, None


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--stop', type=float, default=90.0)
    ap.add_argument('--step', type=float, default=10.0)
    ap.add_argument('--current-limit', type=float, default=None,
                    help='if given, set hand.set_current_limit to this first. '
                         'Take the value from the config (default 0.1) rather '
                         'than inventing one; too high can damage the fingers.')
    ap.add_argument('--no-enable', action='store_true',
                    help='skip hand.set_enable; use if that call blocks')
    ap.add_argument('--enable-timeout', type=float, default=5.0)
    ap.add_argument('--dry-run', action='store_true',
                    help='print the plan and the questions without moving')
    args = ap.parse_args()

    angles = []
    a = args.start
    while a <= args.stop + 1e-9:
        angles.append(round(a, 3))
        a += args.step

    print('=== gripper calibration, stage 1 ===')
    print()
    print('  The gripper has NO readback (probe_gripper --stage 0). This script')
    print('  commands one angle at a time and asks you what you see. Your')
    print('  answers are the calibration data.')
    print()
    print(f'  angles to try: {angles}')
    print()

    if args.dry_run:
        print('  --dry-run: nothing will be commanded.')
        for a in angles:
            print(f'    would set_angle({a}) and ask what the fingers did')
        return 0

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except ImportError as exc:
        print(f'FATAL: cannot import dummy_cli_tool ({exc})')
        return 2

    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first -- only one')
        print('       process can hold the fibre link.')
        return 1
    hand = dev.robot.hand

    # --- optional current limit, before enabling -------------------------
    if args.current_limit is not None:
        print(f'  setting current limit to {args.current_limit} BEFORE '
              'enabling, so the gripper cannot come up at full power')
        _, blocked, msg = with_timeout(
            lambda: hand.set_current_limit(args.current_limit), 8.0,
            f'set_current_limit({args.current_limit})')
        if blocked:
            print(f'    {msg}')
            print('    Stopping: without a current limit set, enabling could')
            print('    energise the fingers at whatever the device defaults to.')
            return 1
        print('    done')
        print()

    # --- enable ----------------------------------------------------------
    if args.no_enable:
        print('  --no-enable: not calling set_enable')
    else:
        print(f'  calling hand.set_enable(True), giving it '
              f'{args.enable_timeout}s')
        _, blocked, msg = with_timeout(lambda: hand.set_enable(True),
                                       args.enable_timeout,
                                       'hand.set_enable(True)')
        if blocked:
            print(f'    {msg}')
            print()
            print('  This is the known failure. It does NOT necessarily mean the')
            print('  gripper is dead: the call may be slow, or may be refused, or')
            print('  may still be pending. The next step tests whether the')
            print('  gripper responds at all, which is the question that matters.')
            print()
            if ask('  Continue and try set_angle anyway? type "yes": ') != 'yes':
                print('  aborted')
                return 1
        else:
            print('    returned')
    print()

    # --- sweep, asking each time ----------------------------------------
    print('  Keep the fingers in view. Nothing between them for this stage.')
    print()
    answers = []
    for a in angles:
        print(f'  --- set_angle({a})')
        _, blocked, msg = with_timeout(lambda a=a: hand.set_angle(a), 8.0,
                                       f'set_angle({a})')
        if blocked:
            print(f'      {msg}')
            print('      The call did not return. Note what the fingers did')
            print('      anyway and record it below; a pending call may still')
            print('      have taken effect.')
        # the fingers move fast; give the eye time to see the final position
        time.sleep(1.5)
        print('      What did the fingers do?')
        print('        o = opened (moved apart)')
        print('        c = closed (moved together)')
        print('        n = no visible change')
        print('        ? = unsure')
        ans = ask('      answer [o/c/n/?]: ').lower()
        note = ask('      optional note (blocked by something, noisy, ...): ')
        answers.append((a, ans, note, msg))
        print()

    # --- summary ---------------------------------------------------------
    print('=' * 70)
    print('ANSWERS')
    print('=' * 70)
    print(f'  {"angle":>8}  {"did":>4}  note')
    for a, ans, note, msg in answers:
        flag = '  <- command blocked' if msg else ''
        print(f'  {a:>8.1f}  {ans or "-":>4}  {note}{flag}')
    print()

    moves = [(a, ans) for a, ans, _, _ in answers if ans in ('o', 'c')]
    if not moves:
        print('  RESULT: the fingers never visibly moved at any angle.')
        print('    That is a real finding, and the most useful thing this run')
        print('    could have produced. Next steps, cheapest first:')
        print('      1. does the gripper have its own power / is it plugged in?')
        print('      2. re-run with --no-enable in case it is already energised')
        print('      3. try set_current_limit at a small non-zero value first')
        print('      4. if set_angle keeps blocking, the gripper channel is not')
        print('         usable and the project needs a different end effector')
        print('         plan. Say so rather than working around it.')
        return 1

    print('  RESULT: the gripper responds. Record these two numbers:')
    opens = [a for a, ans in moves if ans == 'o']
    closes = [a for a, ans in moves if ans == 'c']
    if opens and closes:
        print(f'    fingers OPEN  around angle {min(opens)} .. {max(opens)}')
        print(f'    fingers CLOSE around angle {min(closes)} .. {max(closes)}')
        print()
        print('    If the direction is consistent across the sweep, that gives')
        print('    grip_angle_open / grip_angle_closed for the config. Verify')
        print('    the two end stops by running the sweep again in the opposite')
        print('    direction -- a mapping that is only right one way is a sign')
        print('    error, and this project has been bitten by those repeatedly.')
    else:
        print(f'    open at:  {opens}')
        print(f'    close at: {closes}')
        print('    Only one direction was seen. Widen --start/--stop and re-run')
        print('    to find the other end, or the range may be one-sided.')
    print()
    print('  Next: measure the actual gap between the fingers with a ruler at')
    print('  the two ends, then --stage 2 for the grip force.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
