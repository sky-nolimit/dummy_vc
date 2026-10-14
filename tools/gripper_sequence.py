#!/usr/bin/env python3
"""Walk the gripper interface in one process, so the ORDER is visible.

Why one process
---------------
Testing one call per process cannot distinguish two very different situations:

    a) each call blocks on its own        -- the whole hand channel is broken
    b) the first call blocks and every
       call after it also blocks          -- one call wedges the channel

An earlier run showed `hand.set_angle(90)` blocked. A separate earlier run had
`hand.set_enable(True)` block. Both were the FIRST hand call in their process.
No run has ever made two hand calls, so it is still unknown whether the channel
works at all before it is touched, or whether touching it once is what breaks
it.

This script calls them in order in a single process and prints the outcome of
each, so the pattern shows up directly. Run it in the default order first; if
something blocks, note WHERE in the sequence that happened, because that is the
whole answer.

SAFETY
------
Do not put anything between the fingers. `set_current_limit` is the only thing
that bounds how hard the servo pushes, and on this hardware its units and safe
range are unknown. Set an explicit small limit first, and watch the fingers.

    pkill -f dummy_controller
    python3 tools/gripper_sequence.py
    python3 tools/gripper_sequence.py --order set_current_limit,set_enable,set_angle
    python3 tools/gripper_sequence.py --dry-run
"""
import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))


def call_timed(fn, timeout):
    """Return (state, elapsed, value_or_error)."""
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
        return 'blocked', dt, None
    if 'error' in box:
        return 'raised', dt, box['error']
    return 'ok', dt, box.get('value')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--order', default='set_current_limit,set_enable,set_angle',
                    help='comma separated hand methods, called in this order')
    ap.add_argument('--angle', type=float, default=90.0)
    ap.add_argument('--limit', type=float, default=0.1,
                    help='value for set_current_limit; deliberately small')
    ap.add_argument('--timeout', type=float, default=3.0)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    order = [s.strip() for s in args.order.split(',') if s.strip()]
    known = {'set_angle', 'set_enable', 'set_current_limit'}
    bad = [o for o in order if o not in known]
    if bad:
        print(f'FATAL: unknown methods {bad}; known: {sorted(known)}')
        return 2

    print('=== gripper_sequence ===')
    print()
    print('  call order:', order)
    print(f'  values: set_angle({args.angle}), '
          f'set_current_limit({args.limit}), set_enable(True)')
    print(f'  timeout per call: {args.timeout}s')
    print()
    print('  WATCH THE FINGERS, the two white LEDs on the gripper, and listen')
    print('  for any click or whine. Note the step number where anything')
    print('  changes -- that number is the result.')
    print()

    if args.dry_run:
        print('  --dry-run: nothing will be called.')
        for i, name in enumerate(order, 1):
            print(f'    step {i}: would call hand.{name}(...)')
        return 0

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except Exception as exc:  # noqa: BLE001
        print(f'FATAL: cannot use dummy_cli_tool ({exc})')
        return 2

    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first.')
        return 1
    hand = dev.robot.hand
    print('  connected')
    print()

    args_for = {
        'set_angle': (args.angle, float),
        'set_current_limit': (args.limit, float),
        'set_enable': (True, bool),
    }

    results = []
    wedged_at = None
    for i, name in enumerate(order, 1):
        raw, _cast = args_for[name]
        method = getattr(hand, name, None)
        if method is None:
            print(f'  step {i}: hand.{name} does not exist')
            results.append((i, name, 'missing', 0.0, None))
            continue

        print(f'  step {i}: hand.{name}({raw!r}) ... ', end='', flush=True)
        state, dt, info = call_timed(lambda m=method, v=raw: m(v), args.timeout)
        if state == 'ok':
            print(f'ok  ({dt:.2f}s)')
        elif state == 'blocked':
            print(f'BLOCKED  ({dt:.2f}s)')
            wedged_at = wedged_at if wedged_at is not None else i
        else:
            print(f'raised  ({dt:.2f}s): {info}')
        results.append((i, name, state, dt, info))

        if state == 'blocked':
            print()
            print('         The call has not returned. The daemon thread is')
            print('         still inside it, so the fibre link is held and')
            print('         later steps may fail for that reason rather than')
            print('         their own. Continuing anyway -- the pattern is the')
            print('         point, not any single step.')
            print()

    print()
    print('=' * 70)
    print('SUMMARY')
    print('=' * 70)
    for i, name, state, dt, info in results:
        extra = ''
        if state == 'raised':
            extra = f'  {info}'
        print(f'  step {i}  {name:<20} {state:<8} {dt:5.2f}s{extra}')
    print()

    if wedged_at is None:
        print('  RESULT: every call returned. The hand channel answers.')
        print('    Then the gripper simply did not move, which is a different')
        print('    problem: the command path works and something downstream')
        print('    (servo power, the driver board, the mechanics) does not.')
    elif wedged_at == 1:
        print('  RESULT: the FIRST call blocked.')
        print('    So the channel is unusable before anything is written, and')
        print('    no method is "the bad one". Later steps cannot be trusted')
        print('    in this run.')
        print('    This points at the transport or at the firmware waiting for')
        print('    a reply the gripper board never sends -- not at the')
        print('    semantics of any particular setter.')
    else:
        print(f'  RESULT: the channel worked for {wedged_at - 1} call(s), then')
        print(f'    step {wedged_at} blocked.')
        print('    Steps after it are inconclusive: once the link is held,')
        print('    everything else fails for that reason.')
        print('    Re-run with the blocking method FIRST to see whether it')
        print('    always blocks, or only blocks after another call:')
        print(f'      --order {results[wedged_at - 1][1]},set_current_limit')

    print()
    print('  Either way: report which step number anything visibly happened')
    print('  at. That, plus this table, is what decides the next move.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
