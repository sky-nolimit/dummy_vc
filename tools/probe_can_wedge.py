#!/usr/bin/env python3
"""Does a hand call WEDGE the CAN bus for everything else afterwards?

The hypothesis this tests
-------------------------
The firmware sends CAN frames through one shared function:

    Bsp/communication/interface_can.cpp
        void CanSendMessage(CAN_context* canCtx, uint8_t* txData, ...)
        {
            if (canCtx->handle->Instance == CAN1)
                semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);
            ...
            if (semaphore_status == osOK)
                HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData, ...);
        }

Two things stand out: the acquire waits FOREVER, and the function does not
release the semaphore itself -- some CAN-TX-complete callback is supposed to.
If that callback fails to fire for the gripper's frame, the semaphore never
returns and the firmware is stuck inside CanSendMessage for good. One fault,
explaining everything at once:

  * the hand calls never return            -> the firmware is stuck in the send
  * the client never gets an answer        -> the firmware never reaches the reply
  * the servo never moves                  -> the frame was never transmitted
  * each call costs ~50 s                  -> two endpoint operations, each
                                              exhausting the client's resend budget

The prediction, and it is sharp
-------------------------------
Motor commands go through the SAME function on the SAME bus (CAN1). So if a hand
call wedges it, **every move_j issued afterwards should also stop working** --
including a command that worked a moment earlier.

This script checks exactly that:

    1. move_j away from home, back to home      -> must work
    2. call hand.set_angle (tolerate the wait)  -> the suspected wedge
    3. move_j away from home again              -> works, or wedged?

Outcome A: step 3 still moves the arm
    The hypothesis is WRONG. The bus survives a hand call, so the fault is in
    the hand path itself -- the frame is not reaching the servo, or the servo
    does not answer on that bus. Going direct-to-CAN is then worth doing,
    because it can settle which.

Outcome B: step 3 does not move the arm
    The hypothesis is CONFIRMED, and it is serious: one hand call disables the
    arm until the controller is power-cycled. The driver must then never call
    hand.* -- not even once to look. It is also a clean bug report for the
    vendor: "set_angle wedges CAN TX for all subsequent commands".

SAFETY
------
Moves the arm. The gripper is commanded to 0 degrees, the closed end of the
firmware's 0..30 range, with nothing between the fingers. Keep the power switch
within reach.

    pkill -f dummy_controller
    python3 tools/probe_can_wedge.py
    python3 tools/probe_can_wedge.py --skip-hand      # control run
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from device_session import (connect, move_and_report, power_on,  # noqa: E402
                            wait_settle)

# home_joints_deg is the 7 pose. ARM_A nudges joint_1 by -15 deg, well inside
# its measured accepted range of [-165, 165].
HOME = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
ARM_A = [-15.0, 0.0, 90.0, 0.0, 0.0, 0.0]

HAND_ANGLE = 0.0        # firmware range is 0..30; 0 is the closed end
JUDGE_JOINT = 0         # joint_1 is the one being nudged


def wait_call(fn, timeout):
    """Run fn on a daemon thread and report whether it finished.

    Returns (state, elapsed, detail). The thread is daemonic and never joined
    past the deadline, so a call that never returns cannot stop the script.
    """
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
        time.sleep(1.0)
        if time.monotonic() - t0 >= timeout:
            return 'STILL-RUNNING', time.monotonic() - t0, ''
    th.join(0.3)
    el = time.monotonic() - t0
    if 'exc' in box:
        exc = box['exc']
        return 'RAISED', el, f'{type(exc).__name__}: {exc}'
    return 'RETURNED', el, f'value={box.get("value")!r}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=25.0,
                    help='seconds to wait for the arm to settle (default 25)')
    ap.add_argument('--hand-timeout', type=float, default=60.0,
                    help='seconds to tolerate for the hand call (default 60)')
    ap.add_argument('--skip-hand', action='store_true',
                    help='control run: steps 1 and 3 only, no hand call')
    args = ap.parse_args()

    print('=== probe_can_wedge ===')
    print()
    print('  Question: does one hand call wedge the shared CAN bus, so that')
    print('  motor commands stop working afterwards?')
    print()

    robot, hand = connect()
    if robot is None:
        return 1

    # NOT optional. Without this the axes have no torque, commands are accepted
    # and nothing moves -- which would make every result below meaningless.
    if not power_on(robot):
        print('  The enable call did not return cleanly, so motor motion cannot')
        print('  be trusted. Stopping rather than reporting a false "no move".')
        return 1
    print('  axes enabled (set_enable(True)); without this the arm is limp')
    print()

    print('=' * 74)
    print('STEP 1 -- motor control before anything touches the gripper')
    print('=' * 74)
    try:
        robot.move_j(*HOME)
    except Exception:  # noqa: BLE001
        pass
    wait_settle(robot, args.timeout)
    if not move_and_report(robot, ARM_A, 'move_j -> ARM_A (j1 -15)',
                           args.timeout, index=JUDGE_JOINT):
        print('  The arm did not move BEFORE any hand call.')
        print()
        print('  That is a motor-control problem, not a CAN-wedge result. The')
        print('  most common cause is a missed setup step, and the second most')
        print('  common is that a PREVIOUS hand call already wedged the bus --')
        print('  in which case power-cycle the controller and run this again.')
        return 1
    print('  Baseline is good: the arm responds.')
    print()

    print('=' * 74)
    print('STEP 2 -- one hand call')
    print('=' * 74)
    if args.skip_hand:
        print('  --skip-hand: control run, not touching the hand.')
        print('  Expect step 3 to MOVE. If it does, this harness is sound.')
        print()
    else:
        print(f'  hand.set_angle({HAND_ANGLE})  (firmware range is 0..30)')
        print(f'  tolerating up to {args.hand_timeout:.0f}s')
        print()
        state, el, detail = wait_call(lambda: hand.set_angle(HAND_ANGLE),
                                      args.hand_timeout)
        print(f'    -> {state} after {el:.1f}s')
        if detail:
            print(f'       {detail}')
        if state == 'RETURNED':
            print('    Note: the client does NOT raise after exhausting its')
            print('    retries -- the raise statement sits inside the retry loop')
            print('    and is only reached on a late attempt. The last timeout')
            print('    just falls out of the loop and returns None.')
        print()

    print('=' * 74)
    print('STEP 3 -- can the motor still command, after the hand call?')
    print('=' * 74)
    try:
        robot.move_j(*HOME)
    except Exception:  # noqa: BLE001
        pass
    wait_settle(robot, args.timeout)
    moved_again = move_and_report(robot, ARM_A,
                                  'move_j -> ARM_A again (j1 -15)',
                                  args.timeout, index=JUDGE_JOINT)

    print('=' * 74)
    print('VERDICT')
    print('=' * 74)
    if args.skip_hand:
        if moved_again:
            print('  Control run: the arm moved both times with no hand call,')
            print('  so the harness is sound. A failure after a hand call would')
            print('  therefore mean the hand call is what broke it.')
        else:
            print('  Control run FAILED: the arm stopped moving even without a')
            print('  hand call. Motor control is not reliable enough for this')
            print('  test; something else is wrong.')
        return 0 if moved_again else 1

    if moved_again:
        print('  OUTCOME A: the arm still moves after a hand call.')
        print()
        print('  The shared-CAN-wedge hypothesis is WRONG. The bus survives, so')
        print('  the fault is in the hand path itself: either the frame is not')
        print('  reaching the servo, or the servo does not answer on that bus.')
        print()
        print('  Next: go at the servo directly over CAN. That settles whether')
        print('  anything is listening on node 7 at all, which the firmware')
        print('  path cannot tell us.')
    else:
        print('  OUTCOME B: the arm STOPPED moving after one hand call.')
        print()
        print('  The shared-CAN-wedge hypothesis is CONFIRMED.')
        print()
        print('  One hand call disables motor commands until the controller is')
        print('  power-cycled. That matches the firmware, where CanSendMessage')
        print('  acquires sem_can1_tx with osWaitForever and never releases it:')
        print('  if the TX-complete callback does not fire for the gripper frame,')
        print('  every later send blocks forever.')
        print()
        print('  Consequences, both important:')
        print('    1. the driver must NEVER call hand.* -- one call costs the')
        print('       whole arm until a power cycle')
        print('    2. this is a firmware bug with a clear report for the vendor:')
        print('       "set_angle wedges CAN TX for all subsequent commands"')
    return 0


if __name__ == '__main__':
    sys.exit(main())
