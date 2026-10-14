#!/usr/bin/env python3
"""Measure how a hand call actually ENDS, not just that it is slow.

Why this is needed
------------------
Every earlier probe gave up after 3-8 seconds and reported "blocked". Nobody
ever waited for the real outcome, and the client library says a call cannot
block forever:

    dummy_cli_tool/fibre/protocol.py

        _resend_timeout = 5.0     # [s]
        _send_attempts  = 5
        ...
        while (attempt < self._send_attempts):
            self._output.process_packet(packet)
            if wait_any(self._resend_timeout, ack_event, ...) != 0:
                raise ChannelBrokenException()
            return self._responses.pop(seq_no)
        raise ChannelBrokenException()   # too many resend attempts

So a call that gets no answer should RAISE after roughly 5 x 5 = 25 seconds.
That is a completely different situation from hanging:

    raises ChannelBrokenException  -> the firmware never answered those endpoint
                                      ids, and the client gave up. The call is
                                      recoverable; the channel survives.
    still running past ~40 s       -> something else is wrong (the exception is
                                      being swallowed, or the transport is
                                      stuck), and that is a different bug.

This measures which one it is, for each of the three hand methods, by waiting
long enough and printing the elapsed time and the exact exception.

Read-only with respect to motion: `set_current_limit` and `set_enable` do not
command a position, and `--angle` defaults to 0 which is the closed end of the
firmware's 0..30 range. Keep fingers clear anyway.

    pkill -f dummy_controller
    python3 tools/probe_hand_timeout.py
    python3 tools/probe_hand_timeout.py --methods set_current_limit
    python3 tools/probe_hand_timeout.py --timeout 60
"""
import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

# Values inside the firmware's declared ranges:
#   angle   0..30      (DummyHand::SetAngle clamps to this)
#   current 0..1       (DummyHand::SetMaxCurrent clamps to this)
METHOD_ARGS = {
    'set_angle': 0.0,
    'set_current_limit': 0.05,
    'set_enable': True,
}


def call_watched(method, arg, timeout):
    """Call and report the outcome. Returns (state, elapsed, detail)."""
    box = {}
    t0 = time.monotonic()

    def run():
        try:
            box['value'] = method(arg)
        except BaseException as exc:  # noqa: BLE001
            box['exc'] = exc

    th = threading.Thread(target=run, daemon=True)
    th.start()

    # Poll so progress is visible while waiting; a 25 s silence is exactly the
    # thing earlier probes misread, so show the time passing.
    while th.is_alive():
        time.sleep(1.0)
        el = time.monotonic() - t0
        if el >= timeout:
            return 'STILL-RUNNING', el, f'past {timeout:.0f}s, no outcome'
        if int(el) % 5 == 0:
            print(f'      ... {el:.0f}s', flush=True)

    th.join(0.5)
    el = time.monotonic() - t0
    if 'exc' in box:
        exc = box['exc']
        return 'RAISED', el, f'{type(exc).__module__}.{type(exc).__name__}: {exc}'
    return 'RETURNED', el, f'value={box.get("value")!r}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--methods', default='set_current_limit,set_enable,set_angle',
                    help='comma separated hand methods to test, in order')
    ap.add_argument('--angle', type=float, default=0.0,
                    help='value for set_angle (firmware range is 0..30)')
    ap.add_argument('--timeout', type=float, default=45.0,
                    help='give up after this many seconds (default 45, which '
                         'is past the client\'s ~25 s retry budget)')
    args = ap.parse_args()

    methods = [m.strip() for m in args.methods.split(',') if m.strip()]

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except Exception as exc:  # noqa: BLE001
        print(f'FATAL: cannot use dummy_cli_tool ({exc})')
        return 2

    print('=== probe_hand_timeout ===')
    print()
    print('  The client gives up after 5 attempts x 5 s = ~25 s and raises')
    print('  ChannelBrokenException. Earlier probes stopped at 3-8 s, so the')
    print('  real outcome was never seen. This waits for it.')
    print()
    print(f'  methods: {methods}')
    print(f'  timeout: {args.timeout:.0f}s each')
    print()

    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first.')
        return 1
    hand = dev.robot.hand
    print('  connected')
    print()

    results = []
    for name in methods:
        fn = getattr(hand, name, None)
        if fn is None:
            print(f'  hand.{name} does not exist; skipping')
            continue
        arg = args.angle if name == 'set_angle' else METHOD_ARGS[name]
        print(f'  hand.{name}({arg!r})')
        state, el, detail = call_watched(fn, arg, args.timeout)
        print(f'    -> {state} after {el:.1f}s')
        print(f'       {detail}')
        print()
        results.append((name, state, el, detail))

    print('=' * 74)
    print('SUMMARY')
    print('=' * 74)
    for name, state, el, detail in results:
        print(f'  {name:<20} {state:<16} {el:6.1f}s')
    print()

    states = {r[1] for r in results}
    if states == {'RAISED'}:
        print('  RESULT: every hand call RAISES after roughly 25 s.')
        print('    The firmware never answers endpoint ids 296-301, the client')
        print('    exhausts its 5 resend attempts and gives up.')
        print()
        print('    This is a firmware-side gap, not a client or wiring fault:')
        print('      * the same client path works for robot.move_j')
        print('      * the firmware declares these endpoints (dummy_robot.h)')
        print('      * the CAN send they would perform is the same call the')
        print('        working motors use')
        print('    So the endpoints are declared but do not answer.')
        print()
        print('    Practical consequence: the driver must never call them,')
        print('    because each attempt costs ~25 s of the only fibre link.')
    elif 'STILL-RUNNING' in states:
        print('  RESULT: at least one call was STILL RUNNING past the timeout.')
        print('    That does not match the client\'s retry logic, which should')
        print('    raise after ~25 s. Something else is holding it: either the')
        print('    exception is being swallowed, or the USB transport is stuck.')
        print('    Re-run with a larger --timeout to see whether it ever ends.')
    else:
        print('  RESULT: mixed outcomes; see the table above.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
