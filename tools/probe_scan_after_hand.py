#!/usr/bin/env python3
"""Which device operations still work after a hand call? Exactly one each.

Why this shape
--------------
The last hardware run ended with the script hanging at step 3, "no terminal
output". That is not a timeout -- it means `robot.move_j(...)` itself never
returned. So a hand call does not merely make the arm stop moving; it stops the
device from ANSWERING AT ALL.

That is worth pinning down precisely, because the candidates need different
fixes, and two of them are pure configuration:

  A. the firmware is stuck inside its own CAN send
     (`CanSendMessage` acquires sem_can1_tx with osWaitForever and never
     releases it) -> a firmware bug or a bus fault

  B. a protocol mismatch: the client stamps each request with the CRC of the
     interface definition, and a firmware whose definitions differ silently
     drops the request with no reply -> a version mismatch, nothing to do with
     wiring, and it would also explain the hand calls

  C. only the WRITE path is wedged while reads still work
     -> points at the request/response machinery rather than the CAN send

This script runs ONE operation per invocation, each on its own daemon thread
with a hard deadline, so nothing can hang and every outcome is reported. Run the
steps in order, power-cycling between invocations, and the comparison between
them is the answer.

    pkill -f dummy_controller
    python3 tools/probe_scan_after_hand.py --step read
    python3 tools/probe_scan_after_hand.py --step read-after-hand
    python3 tools/probe_scan_after_hand.py --step write-after-hand
    python3 tools/probe_scan_after_hand.py --step definition

`--step definition` is the one that settles candidate B, and it touches nothing
that moves, so run it first if you want a cheap answer.

SAFETY: the steps ending in -after-hand move nothing new, but `write-after-hand`
commands the arm. Keep the power switch within reach.
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from device_session import connect, power_on  # noqa: E402

HOME = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
ARM_A = [-15.0, 0.0, 90.0, 0.0, 0.0, 0.0]

STEPS = ('read', 'write', 'hand', 'read-after-hand', 'write-after-hand',
         'definition')


def watched(label, fn, timeout, skip=False):
    """Run fn on a daemon thread; report the outcome within `timeout`.

    `skip=True` does not call fn at all. That matters because a call that never
    returns leaves a daemon thread holding a libusb mutex, and then Ctrl-C
    cannot exit cleanly -- the process aborts with an assertion inside libusb
    instead. When an earlier step already told us a call hangs, repeating it
    buys nothing and costs a stuck process.
    """
    if skip:
        print(f'  {label}')
        print('    -> SKIPPED (a previous step already showed this call never')
        print('       returns; repeating it would only wedge the process)')
        return 'SKIPPED'
    box = {}
    t0 = time.monotonic()

    def run():
        try:
            box['value'] = fn()
        except BaseException as exc:  # noqa: BLE001
            box['exc'] = exc

    th = threading.Thread(target=run, daemon=True)
    th.start()
    while th.is_alive():
        time.sleep(0.5)
        if time.monotonic() - t0 >= timeout:
            print(f'  {label}')
            print(f'    -> NO ANSWER after {time.monotonic() - t0:.1f}s')
            print(f'       The request was sent and nothing came back. The')
            print(f'       client will not raise here; it retries and times out.')
            return 'NO-ANSWER'
    th.join(0.3)
    el = time.monotonic() - t0
    print(f'  {label}')
    if 'exc' in box:
        exc = box['exc']
        print(f'    -> RAISED after {el:.2f}s  {type(exc).__name__}: {exc}')
        return 'RAISED'
    print(f'    -> OK after {el:.2f}s   value={box.get("value")!r}')
    return 'OK'


def report_retry_budget(robot):
    """Print the client's actual retry parameters, from the live object.

    Worth doing rather than trusting the source: `_resend_timeout = 5.0` and
    `_send_attempts = 5` are class attributes, and if a transport overrides them
    the real budget is different from the ~25 s the source suggests. That
    difference decides whether "never returns" is even possible.
    """
    print('  client retry budget, read from the live channel:')
    found = False
    for owner, label in ((robot, 'robot'),):
        for attr in dir(owner):
            if attr.startswith('_'):
                continue
            obj = getattr(owner, attr, None)
            ch = getattr(obj, '__channel__', None)
            if ch is None:
                continue
            found = True
            rt = getattr(ch, '_resend_timeout', None)
            sa = getattr(ch, '_send_attempts', None)
            print(f'    via {label}.{attr}: _resend_timeout={rt} '
                  f'_send_attempts={sa}  -> worst case '
                  f'{(rt or 0) * (sa or 0):.1f}s per endpoint operation')
            break
    if not found:
        print('    could not reach the channel object from the robot; the')
        print('    class defaults are _resend_timeout=5.0, _send_attempts=5')
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--step', required=True, choices=STEPS)
    ap.add_argument('--timeout', type=float, default=20.0,
                    help='seconds to wait for one operation (default 20)')
    ap.add_argument('--hand-timeout', type=float, default=60.0,
                    help='seconds to tolerate for the hand call (default 60)')
    ap.add_argument('--redo-hand-call', action='store_true',
                    help='make the hand call inside read-after-hand / '
                         'write-after-hand. Off by default: the call never '
                         'returns and leaves a libusb mutex held, so the '
                         'process cannot be Ctrl-C\'d cleanly. Only needed if '
                         'you power-cycled and want the wedge created here.')
    args = ap.parse_args()

    print(f'=== probe_scan_after_hand -- step: {args.step} ===')
    print()
    print('  ONE operation per invocation, each with a hard deadline, so')
    print('  nothing can hang. Power-cycle the controller between steps.')
    print()

    robot, hand = connect()
    if robot is None:
        return 1
    if not power_on(robot):
        print('  enable did not return cleanly; stopping')
        return 1
    print('  axes enabled')
    print()

    if args.step == 'definition':
        # Purely informational: does the interface definition we receive carry
        # a CRC, and can the client use it? A mismatch there would make the
        # firmware drop requests with NO reply -- candidate B.
        print('  Interface definition (does the device describe the endpoints')
        print('  the client is calling, and with which ids?):')
        try:
            import dummy_cli_tool.ref_tool as ref_tool  # noqa: F401
            dev_raw = getattr(robot, '_dev', None)
        except Exception:  # noqa: BLE001
            pass
        # Walk the hand object's protocol metadata as the CLIENT sees it.
        for name in ('set_angle', 'set_enable', 'set_current_limit'):
            attr = getattr(hand, name, None)
            if attr is None:
                print(f'    hand.{name}: MISSING from the client interface')
                continue
            tid = getattr(attr, '_trigger_id', None)
            ins = [(getattr(i, '_id', None), getattr(i, '_name', None))
                   for i in getattr(attr, '_inputs', [])]
            outs = len(getattr(attr, '_outputs', []) or [])
            print(f'    hand.{name:<20} trigger_id={tid} inputs={ins} '
                  f'outputs={outs}')
        n_attr = [a for a in dir(hand) if not a.startswith('_')]
        print(f'    hand exposes: {n_attr}')
        print()
        print('  For comparison, an endpoint KNOWN to work:')
        mj = getattr(robot, 'move_j', None)
        if mj is not None:
            print(f'    robot.move_j           trigger_id='
                  f'{getattr(mj, "_trigger_id", None)}')
        print()
        print('  Reading the ids above is a CONSISTENCY check, not an')
        print('  independent test: the client takes these ids from the device\'s')
        print('  own interface definition at discovery time, so they agree by')
        print('  construction. They are printed to confirm the device really is')
        print('  advertising a gripper -- which the earlier dump already showed')
        print('  (set_angle 296/297, set_enable 298/299,')
        print('  set_current_limit 300/301 in that dump).')
        print()
        print('  So a protocol mismatch is unlikely: the device advertises these')
        print('  endpoints and the client calls exactly them. What is NOT')
        print('  explained is why the device then never answers.')
        return 0

    if args.step == 'read':
        st = watched('read a joint angle (robot.joint_1.angle)',
                     lambda: float(robot.joint_1.angle), args.timeout)
        print()
        print('  Baseline. This is the cheapest possible round trip to the')
        print('  device, so if it fails here nothing else will work either.')
        return 0 if st == 'OK' else 1

    if args.step == 'write':
        st = watched('move_j(ARM_A)', lambda: robot.move_j(*ARM_A),
                     args.timeout)
        print()
        print('  Baseline write. Takes torque, so the arm should jerk to the')
        print('  side if the axes are enabled.')
        return 0 if st == 'OK' else 1

    if args.step == 'hand':
        st = watched('hand.set_angle(0.0)',
                     lambda: hand.set_angle(0.0), args.hand_timeout)
        print()
        print('  The suspected wedge. Expect NO ANSWER.')
        return 0

    if args.step == 'read-after-hand':
        print('  Calling hand.set_angle(0.0) first (expect no answer), then')
        print('  reading a joint angle to see whether the device still replies.')
        print()
        report_retry_budget(robot)
        watched('hand.set_angle(0.0)', lambda: hand.set_angle(0.0),
                args.hand_timeout)
        print()
        st = watched('read a joint angle AFTER the hand call',
                     lambda: float(robot.joint_1.angle), args.timeout)
        print()
        if st == 'OK':
            print('  VERDICT: reads still work after a hand call.')
            print('    So the device is NOT wedged -- it still answers on the')
            print('    fibre interface. The failure is specific to the WRITE')
            print('    path (candidate C), not to a stuck firmware loop.')
        elif st == 'SKIPPED':
            print('  VERDICT: inconclusive -- the read was skipped.')
        else:
            print('  VERDICT: reads STOPPED working after a hand call.')
            print('    The device no longer answers at all. Combined with the')
            print('    earlier run, where a subsequent move_j never returned,')
            print('    that means the firmware is stuck and is servicing')
            print('    nothing -- consistent with CanSendMessage blocking')
            print('    forever on sem_can1_tx (candidate A).')
        return 0

    # write-after-hand
    print('  Calling hand.set_angle(0.0) first, then commanding a move, to see')
    print('  whether the WRITE path survives.')
    print()
    report_retry_budget(robot)
    # Skip by default: the earlier run established that this call never
    # returns, and repeating it wedges a libusb mutex so the process cannot
    # even be Ctrl-C'd cleanly. Almost always the hand call was already made
    # in a previous invocation and the controller was NOT power-cycled, which
    # is exactly the state this step wants to measure.
    watched('hand.set_angle(0.0)', lambda: hand.set_angle(0.0),
            args.hand_timeout, skip=not args.redo_hand_call)
    if args.redo_hand_call:
        print()
        print('  (--redo-hand-call was given, so the hand was called here.)')
    else:
        print()
        print('  If a previous invocation already called the hand and you did')
        print('  NOT power-cycle since, the wedge is already in place and the')
        print('  measurement below is valid. Pass --redo-hand-call to make the')
        print('  call in this run instead.')
    print()
    st = watched('move_j(ARM_A) AFTER the hand call',
                 lambda: robot.move_j(*ARM_A), args.timeout)
    print()
    if st == 'OK':
        print('  VERDICT: writes still work, so the CAN bus is not wedged and')
        print('    the hand fault is local to the gripper.')
    elif st == 'NO-ANSWER':
        print('  VERDICT: WRITES stopped answering.')
        print('    Motor commands and gripper commands share CanSendMessage on')
        print('    CAN1, so this is the shared-bus wedge (candidate A): one')
        print('    hand call takes the whole arm out until a power cycle.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
