#!/usr/bin/env python3
"""Call exactly ONE gripper method, and report what happened. Nothing else.

Why this shape
--------------
The gripper is a black box. What is known is only:

    hand has exactly three methods -- set_angle, set_current_limit, set_enable
    -- and no readable attribute at all (probe_gripper --stage 0, on hardware)
    hand.set_enable(True) did not return within 8 s

An earlier probe called set_enable and then set_angle in the same run. When
set_enable hung, nothing about set_angle was learned -- and set_angle is the
call that actually matters. One variable per run avoids that completely: if a
run hangs, it says exactly which call hangs.

Each invocation also uses a SHORT timeout, so a blocked call is reported in
seconds instead of freezing for a long one.

    pkill -f dummy_controller

    python3 tools/gripper_call.py --call none
    python3 tools/gripper_call.py --call set_angle --value 90
    python3 tools/gripper_call.py --call set_current_limit --value 0.1
    python3 tools/gripper_call.py --call set_enable --value 1

Watch the fingers and the green LED on the gripper board during each run, and
report both. "The LED changed" is data; so is "the fingers twitched once".

Safety: set_current_limit is the only thing that limits how hard the servo
pushes. Until it is understood, do not put anything between the fingers.
"""
import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

CALLS = ('none', 'set_angle', 'set_current_limit', 'set_enable')


def call_with_timeout(fn, timeout):
    """Run fn on a daemon thread. Return (value, state, seconds).

    state is 'ok', 'blocked' or 'raised'. The thread is daemonic and is never
    joined past the deadline, so a call that never returns does not stop the
    probe from reporting.
    """
    box = {}
    t0 = time.monotonic()

    def run():
        try:
            box['value'] = fn()
        except Exception as exc:  # noqa: BLE001
            box['error'] = exc

    th = threading.Thread(target=run, daemon=True)
    th.start()
    th.join(timeout)
    dt = time.monotonic() - t0
    if th.is_alive():
        return None, 'blocked', dt
    if 'error' in box:
        return None, f'raised:{type(box["error"]).__name__}', dt, box['error']
    return box.get('value'), 'ok', dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--call', required=True, choices=CALLS,
                    help='"none" connects and reads only, to establish a '
                         'baseline before anything is written')
    ap.add_argument('--value', default=None,
                    help='argument for the call: degrees for set_angle, the '
                         'limit for set_current_limit, 0 or 1 for set_enable')
    ap.add_argument('--timeout', type=float, default=3.0,
                    help='seconds to wait before declaring the call blocked '
                         '(default 3; short on purpose)')
    args = ap.parse_args()

    if args.call != 'none' and args.value is None:
        print(f'FATAL: --call {args.call} needs --value')
        return 2

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except Exception as exc:  # noqa: BLE001
        # Not just ImportError: the module needs libusb and a USB backend, and
        # on a machine without them the import fails in other ways. Catch
        # broadly so the message is a clear FATAL instead of a traceback.
        print(f'FATAL: cannot use dummy_cli_tool on this machine ({exc})')
        print('       This probe needs the arm connected and the vendor '
              'library present; run it on the machine that talks to the arm.')
        return 2

    print('=== gripper_call ===')
    print()
    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first -- only one')
        print('       process can hold the fibre link.')
        return 1
    print('  device found')
    hand = dev.robot.hand

    if args.call == 'none':
        print()
        print('  --call none: connected and took the hand handle. No call was')
        print('  made, so this establishes that the handle itself is fine')
        print('  before any write is attempted.')
        print()
        print('  hand methods:',
              [n for n in dir(hand) if not n.startswith('_')])
        return 0

    method = getattr(hand, args.call, None)
    if method is None:
        print(f'FATAL: hand has no method {args.call}')
        return 2

    if args.call == 'set_enable':
        value = args.value.strip().lower() in ('1', 'true', 'yes', 'on')
    else:
        value = float(args.value)

    print(f'  calling hand.{args.call}({value!r}), timeout {args.timeout}s')
    print()
    print('  WATCH THE GRIPPER while this runs and note:')
    print('    * did the fingers move? which way? how far roughly?')
    print('    * did the green LED on the gripper board change?')
    print('    * any noise, twitch or click?')
    print()

    out = call_with_timeout(lambda: method(value), args.timeout)
    if len(out) == 3:
        result, state, dt = out
        err = None
    else:
        result, state, dt, err = out

    print(f'  hand.{args.call}({value!r})')
    print(f'    state  : {state}')
    print(f'    elapsed: {dt:.2f} s')
    if result is not None:
        print(f'    returned: {result!r}')
    if err is not None:
        print(f'    error  : {err}')

    print()
    if state == 'ok':
        print('  The call returned. That is one fact established: this method')
        print('  does not block.')
    elif state == 'blocked':
        print('  The call did NOT return. Two things follow:')
        print('    1. this method is the one that blocks, so anything that')
        print('       calls it (including the driver\'s gripper init) will hang')
        print('       the fibre link until the process dies')
        print('    2. whether it still had an effect is unknown -- a pending')
        print('       call may already have moved the servo. That is what your')
        print('       observation of the fingers is for.')
    else:
        print('  The call raised rather than blocking. That is much better')
        print('  news than a block: the interface exists and answers, and the')
        print('  message above says what it objected to.')

    print()
    print('  NEXT: run again with a different single call. Change only one')
    print('  thing per run, so the result is attributable.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
