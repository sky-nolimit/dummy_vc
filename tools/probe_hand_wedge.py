#!/usr/bin/env python3
"""Is it the CLAW that drags the CAN bus down, or the firmware alone?

The question this answers
-------------------------
Calling any `hand.*` method kills the whole arm until a power cycle. Reading
`Bsp/communication/interface_can.cpp` explains HOW the whole arm can die from
one bad frame:

    CanSendMessage()      osSemaphoreAcquire(sem_can1_tx, osWaitForever)
                          ... HAL_CAN_AddTxMessage(...)   // return value ignored
    tx_complete_callback  osSemaphoreRelease(sem_can1_tx) // the ONLY release
    HAL_CAN_ErrorCallback TX errors / arbitration loss: no release at all

So one frame that never reaches "transmitted" loses that semaphore forever, and
because the periodic joint update calls the same function, the arm stops
answering on the fibre link too. That matches every measurement so far.

What is still unknown is WHY the claw frame never completes. The cheap
discriminator is mechanical:

    unplug the claw from the wrist, power-cycle, call hand.set_angle(0.0)

  still wedges  -> the claw's presence never mattered. The frame is not being
                   acknowledged because node 7 is not on that bus at all, or
                   the firmware puts it on a different bus. Look at wiring or
                   at get_can_ctx(), not at the servo.
  no longer wedges (returns, or raises) -> the claw itself is dragging the bus
                   into an error state (bitrate, H/L swapped, no termination,
                   or the servo jamming). Check the wiring and the bitrate.

Why this runs one phase per PROCESS
-----------------------------------
A client call that never returns leaves a daemon thread holding a libusb mutex,
so a follow-up call in the same process is blocked by the CLIENT, not by the
device -- and that looks identical to "the board is dead". Earlier probes read
that as a firmware finding. So each phase is its own process, and the parent
records only the exit status and the child's last words.

    pkill -f dummy_controller
    python3 tools/probe_hand_wedge.py --claw plugged      # baseline run
    # power-cycle, unplug the claw, then:
    python3 tools/probe_hand_wedge.py --claw unplugged    # the experiment

SAFETY: no motion is commanded. `set_enable(True)` applies torque (that is why
the fingers stiffen), so park the arm first. If a phase wedges the board you
must power-cycle -- the arm stays where it is.
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Exit codes the child phases use to tell the parent what happened.
RC_OK = 0
RC_FAIL = 2
RC_HAND_RAISED = 3
RC_NO_DEVICE = 4

PHASES = ('baseline', 'hand', 'after')


def phase_baseline():
    """Can we talk to it at all right now? Cheapest possible round trip."""
    from device_session import connect, power_on, raw_angles
    dev, hand = connect()
    if dev is None:
        return RC_NO_DEVICE
    if not power_on(dev):
        print('  set_enable(True) did not return cleanly', flush=True)
        return RC_FAIL
    print(f'  angles: {["%+.3f" % a for a in raw_angles(dev)]}', flush=True)
    print('  BASELINE-OK', flush=True)
    return RC_OK


def phase_hand():
    """The suspected wedge. The parent kills this process if it hangs."""
    from device_session import connect, power_on
    dev, hand = connect()
    if dev is None:
        return RC_NO_DEVICE
    if not power_on(dev):
        print('  set_enable(True) did not return cleanly', flush=True)
        return RC_FAIL
    print('  calling hand.set_angle(0.0)', flush=True)
    t0 = time.monotonic()
    try:
        hand.set_angle(0.0)
    except BaseException as exc:  # noqa: BLE001
        print(f'  RAISED after {time.monotonic() - t0:.1f}s '
              f'{type(exc).__name__}: {exc}', flush=True)
        return RC_HAND_RAISED
    print(f'  RETURNED after {time.monotonic() - t0:.1f}s', flush=True)
    return RC_OK


def phase_after():
    """Fresh process, fresh libusb handle: is the board still answering?"""
    from device_session import connect, power_on, raw_angles
    dev, hand = connect()
    if dev is None:
        return RC_NO_DEVICE
    if not power_on(dev):
        print('  set_enable(True) did not return: board is not answering',
              flush=True)
        return RC_FAIL
    print(f'  angles: {["%+.3f" % a for a in raw_angles(dev)]}', flush=True)
    print('  ALIVE', flush=True)
    return RC_OK


def run_phase(name, timeout):
    """Run one phase in a child process. Returns (state, rc, elapsed, output)."""
    cmd = [sys.executable, os.path.abspath(__file__), '--phase', name]
    t0 = time.monotonic()
    try:
        proc = subprocess.run(cmd, timeout=timeout, capture_output=True,
                              text=True, errors='replace')
    except subprocess.TimeoutExpired as exc:
        elapsed = time.monotonic() - t0
        out = (exc.stdout or '') + (exc.stderr or '')
        if isinstance(out, bytes):
            out = out.decode('utf-8', 'replace')
        return 'HUNG', None, elapsed, out
    elapsed = time.monotonic() - t0
    out = (proc.stdout or '') + (proc.stderr or '')
    if proc.returncode == RC_OK:
        return 'OK', proc.returncode, elapsed, out
    if proc.returncode == RC_HAND_RAISED:
        return 'RAISED', proc.returncode, elapsed, out
    return 'FAILED', proc.returncode, elapsed, out


def show(state, elapsed, out):
    print(f'    -> {state} after {elapsed:.1f}s')
    for line in out.splitlines():
        line = line.rstrip()
        if line:
            print(f'       | {line}')
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--phase', choices=PHASES, default=None,
                    help='internal: run one phase (the parent does this)')
    ap.add_argument('--claw', choices=('plugged', 'unplugged'), default=None,
                    help='how the claw is wired for THIS run; only used to '
                         'phrase the verdict')
    ap.add_argument('--hand-timeout', type=float, default=40.0,
                    help='how long to tolerate hand.set_angle before calling '
                         'it hung (default 40, past the client\'s ~25 s budget)')
    ap.add_argument('--phase-timeout', type=float, default=60.0,
                    help='deadline for the baseline / after phases (default 60)')
    ap.add_argument('--pause', type=float, default=3.0,
                    help='seconds between phases to let USB settle (default 3)')
    args = ap.parse_args()

    # Child mode: the parent invoked us for one phase.
    if args.phase == 'baseline':
        return phase_baseline()
    if args.phase == 'hand':
        return phase_hand()
    if args.phase == 'after':
        return phase_after()

    if args.claw is None:
        print('FATAL: --claw plugged|unplugged is required for a full run')
        return 2

    print('=== probe_hand_wedge ===')
    print()
    print(f'  claw wiring for this run: {args.claw}')
    print('  three phases, each its own process, so a wedged client thread')
    print('  cannot be mistaken for a wedged device.')
    print()

    states = {}
    for name in PHASES:
        label, timeout = {
            'baseline': ('PHASE 1/3  baseline: read the six angles',
                         args.phase_timeout),
            'hand': ('PHASE 2/3  the suspected wedge: hand.set_angle(0.0)',
                     args.hand_timeout),
            'after': ('PHASE 3/3  fresh process: does the board still answer?',
                      args.phase_timeout),
        }[name]
        print(label)
        state, rc, elapsed, out = run_phase(name, timeout)
        show(state, elapsed, out)
        states[name] = state
        if name == 'baseline' and state != 'OK':
            if rc == RC_NO_DEVICE:
                print('  VERDICT: inconclusive -- no device (or the client')
                print('    library is not importable here). This run proves')
                print('    nothing; fix the connection and start again.')
            elif state == 'HUNG':
                print('  VERDICT: the board was already not answering before')
                print('    the hand call. Power-cycle, then re-run before')
                print('    drawing any conclusion.')
            else:
                print('  VERDICT: inconclusive -- the baseline itself failed.')
                print('    See the phase output above; do not read the later')
                print('    phases as a result.')
            return 1
        if name != 'after':
            time.sleep(max(0.0, args.pause))

    base, hand, after = states['baseline'], states['hand'], states['after']

    print('=' * 74)
    print('RESULT')
    print('=' * 74)
    print(f'  baseline (read angles)        {base}')
    print(f'  hand.set_angle(0.0)           {hand}')
    print(f'  board answers afterwards      {after}')
    print()

    if hand == 'HUNG' and after != 'OK':
        print('  VERDICT: the wedge reproduced -- one hand frame still takes')
        print('    the whole CAN path (and therefore the whole board) out.')
        if args.claw == 'unplugged':
            print()
            print('    The claw was UNPLUGGED and nothing changed. So the claw')
            print('    is not what drags the bus down: nobody was ever')
            print('    acknowledging that frame in the first place. The frame')
            print('    is either going to a bus node 7 is not on, or the')
            print('    firmware is sending it somewhere else. Next step is')
            print('    wiring / get_can_ctx(), not the servo.')
        else:
            print()
            print('    Baseline for comparison with --claw unplugged.')
    elif hand == 'HUNG' and after == 'OK':
        print('  VERDICT: the hand call hung but the board still answers after')
        print('    a reconnect. So this is NOT the shared-semaphore death: the')
        print('    client/transport is what stops, and the device is alive.')
        print('    That points at the request/response path for the hand')
        print('    endpoints, not at the CAN send.')
    elif hand in ('OK', 'RAISED') and after == 'OK':
        print('  VERDICT: the hand call ended and the board survived.')
        print('    The CAN path was never lost, so this frame is not the one')
        print('    that kills the board. If it RAISED, the device never')
        print('    answered the endpoint -- a firmware gap, not wiring.')
        if args.claw == 'unplugged':
            print()
            print('    Claw was unplugged, so "no answer" is expected: node 7')
            print('    is not there. The finding is that its absence does not')
            print('    wedge the board, which points at the claw itself (or its')
            print('    wiring) being what wedges the bus when plugged.')
    else:
        print('  VERDICT: mixed; see the phase lines above.')

    print()
    if after != 'OK':
        print('  The board is wedged now: power-cycle it before using the arm.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
