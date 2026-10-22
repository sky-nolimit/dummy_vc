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
import contextlib
import os
import subprocess
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Exit codes the child phases use to tell the parent what happened.
RC_OK = 0
RC_FAIL = 2
RC_HAND_RAISED = 3
RC_NO_DEVICE = 4

PHASES = ('baseline', 'hand', 'after')


@contextlib.contextmanager
def heartbeat(label):
    """Print "still waiting" every 5 s while a blocking SDK call runs.

    A silent terminal is the whole problem this script exists to avoid: a call
    that blocks for 25+ s looks the same as a dead process. This timer runs in
    the CHILD, so even if the parent is gone the screen still says the call has
    not returned.
    """
    stop = threading.Event()

    def run():
        waited = 0
        while not stop.wait(5.0):
            waited += 5
            print(f'  ... {label}: still no return after {waited}s', flush=True)

    th = threading.Thread(target=run, daemon=True)
    th.start()
    try:
        yield
    finally:
        stop.set()


def phase_baseline():
    """Can we talk to it at all right now? Cheapest possible round trip."""
    from device_session import connect, power_on, raw_angles
    with heartbeat('connect + enable'):
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
    with heartbeat('connect + enable'):
        dev, hand = connect()
        if dev is None:
            return RC_NO_DEVICE
        if not power_on(dev):
            print('  set_enable(True) did not return cleanly', flush=True)
            return RC_FAIL
    print('  calling hand.set_angle(0.0)', flush=True)
    t0 = time.monotonic()
    try:
        with heartbeat('hand.set_angle'):
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
    with heartbeat('connect + enable'):
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
    """Run one phase as a child that inherits our stdio, so its output is live.

    Live output is the point. With `capture_output` a phase that hangs looks
    exactly like a parent that hangs, and the run reads as "froze, then exited".
    So the deadline is announced before the wait and a dot is printed every two
    seconds while it runs: "still waiting" and "given up" must not look alike.

    Returns (state, returncode, elapsed).
    """
    cmd = [sys.executable, os.path.abspath(__file__), '--phase', name]
    print(f'    (deadline {timeout:.0f}s; a dot every 2s = still waiting)',
          flush=True)
    t0 = time.monotonic()
    # start_new_session: the child must not share our foreground process group.
    # Otherwise one Ctrl-C takes the parent down mid-measurement, the run ends
    # with no verdict, and that looks exactly like "the script died on its own".
    proc = subprocess.Popen(cmd, start_new_session=True)  # inherits stdio
    last_dot = 0.0
    try:
        while True:
            rc = proc.poll()
            el = time.monotonic() - t0
            if rc is not None:
                print(flush=True)
                break
            if el >= timeout:
                proc.kill()
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    print('    (the child did not die after SIGKILL)',
                          flush=True)
                print(flush=True)
                return 'HUNG', None, el
            if el - last_dot >= 2.0:
                last_dot = el
                print('.', end='', flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        # Keep the measurement honest: kill the child, say so, and let the
        # caller decide. Never leave a silent corpse.
        proc.kill()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            pass
        print(flush=True)
        return 'INTERRUPTED', None, time.monotonic() - t0
    if rc == RC_OK:
        return 'OK', rc, el
    if rc == RC_HAND_RAISED:
        return 'RAISED', rc, el
    return 'FAILED', rc, el


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
    rcs = {}
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
        state, rc, elapsed = run_phase(name, timeout)
        print(f'    -> {state} after {elapsed:.1f}s')
        print()
        states[name] = state
        rcs[name] = rc
        if state == 'INTERRUPTED':
            print('  The phase was killed by your Ctrl-C, so the run stops')
            print('  here and proves nothing more. If it was phase 2 that was')
            print('  cut short, that does NOT mean the board survived --')
            print('  power-cycle it and run again with the output saved:')
            print('    python3 tools/probe_hand_wedge.py --claw unplugged \\')
            print('      2>&1 | tee /tmp/wedge.txt')
            return 130
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
        if rcs.get('after') == RC_NO_DEVICE:
            print()
            print('    Note: phase 3 did not merely hang on a call, it could')
            print('    not even finish USB discovery. The controller is not')
            print('    servicing its USB interface at all, which is stronger')
            print('    evidence than a single blocked request.')
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
    # Always leave an EXIT= line. A run that ends with no message at all is
    # indistinguishable from a run that was killed, and that ambiguity has
    # already cost one measurement here.
    try:
        code = main()
    except KeyboardInterrupt:
        print('\n  INTERRUPTED (Ctrl-C).', flush=True)
        code = 130
    except BaseException:  # noqa: BLE001
        traceback.print_exc()
        code = 1
    print(f'EXIT={code}', flush=True)
    sys.exit(code)
