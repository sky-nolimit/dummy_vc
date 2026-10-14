#!/usr/bin/env python3
"""Does a hand call WEDGE the CAN bus for everything else afterwards?

The hypothesis this tests
-------------------------
The firmware sends CAN frames through one shared function:

    interface_can.cpp
        void CanSendMessage(CAN_context* canCtx, uint8_t* txData, ...)
        {
            if (canCtx->handle->Instance == CAN1)
                semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);
            ...
            if (semaphore_status == osOK)
                HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData, ...);
        }

Note two things: the acquire waits FOREVER, and the function does not release
the semaphore itself -- some CAN-TX-complete callback is supposed to.

If that callback fails to fire for the gripper's frame, the semaphore never
comes back and the firmware is stuck inside CanSendMessage for good. That single
fault would explain every observation at once:

  * the hand calls never return            -> the firmware is stuck in the send
  * the client never gets an answer        -> the firmware never reaches the reply
  * the servo never moves                  -> the frame was never transmitted
  * each call costs ~50 s                  -> two endpoint operations, each
                                              exhausting the client's resend budget

The prediction, and it is a sharp one
-------------------------------------
Motor commands go through the SAME function on the SAME bus (CAN1). So if a hand
call wedges it, then **every move_j issued afterwards should also stop working**
-- and it should stop working for a command that worked a moment earlier.

This script checks exactly that:

    1. move_j away from home, back to home      -> must work
    2. call hand.set_angle (tolerate the wait)  -> the suspected wedge
    3. move_j away from home again              -> works, or wedged?

Outcome A: step 3 still moves the arm
    The hypothesis is WRONG. The CAN bus survives a hand call, so the fault is
    somewhere in the hand path itself, not in shared bus contention.

Outcome B: step 3 does not move the arm
    The hypothesis is CONFIRMED, and it is serious: one hand call permanently
    disables the arm until the controller is power-cycled. That also means the
    driver must never call hand.* -- not even once for a look.

Either answer is worth having, and this is the cheapest way to get it.

SAFETY
------
Moves the arm. The gripper is commanded to 0 degrees, the closed end of the
firmware's 0..30 range, with nothing between the fingers. Keep the power switch
within reach.

    pkill -f dummy_controller
    python3 tools/probe_can_wedge.py
    python3 tools/probe_can_wedge.py --skip-hand      # control: steps 1 and 3 only
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# home_joints_deg: the 7 pose. ARM_A nudges joint_1 by 15 deg, which is well
# inside its measured accepted range of [-165, 165].
HOME = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
ARM_A = [-15.0, 0.0, 90.0, 0.0, 0.0, 0.0]

HAND_ANGLE = 0.0        # firmware range is 0..30; 0 is the closed end


def raw(robot):
    return [float(getattr(robot, n).angle) for n in JR]


def wait_settle(robot, timeout=25.0, eps=0.05, quiet=0.8):
    t0 = time.monotonic()
    last = raw(robot)
    stable = time.monotonic()
    while time.monotonic() - t0 < timeout:
        time.sleep(0.2)
        now = raw(robot)
        if max(abs(now[i] - last[i]) for i in range(6)) > eps:
            stable = time.monotonic()
        elif time.monotonic() - stable > quiet:
            return now, time.monotonic() - t0, False
        last = now
    return raw(robot), time.monotonic() - t0, True


def wait_call(fn, timeout):
    """Run fn on a daemon thread and report whether it finished.

    Returns (state, elapsed, detail). `state` is 'RETURNED', 'RAISED' or
    'STILL-RUNNING'. The thread is daemonic and never joined past the deadline,
    so a call that never returns cannot stop the script.
    """
    import threading
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


def move_test(robot, target, label, timeout):
    """Command `target` and report whether the arm actually moved."""
    before = raw(robot)
    try:
        robot.move_j(*target)
        err = None
    except Exception as exc:  # noqa: BLE001
        err = f'{type(exc).__name__}: {exc}'
    after, elapsed, trunc = wait_settle(robot, timeout)
    # joint_1 is the one being nudged; use it as the indicator
    moved = abs(after[0] - before[0])
    print(f'  {label}')
    print(f'    before j1 = {before[0]:+8.3f}')
    print(f'    after  j1 = {after[0]:+8.3f}   (moved {moved:.3f} deg, '
          f'{elapsed:.1f}s{"  WAIT TRUNCATED" if trunc else ""})')
    if err:
        print(f'    RAISED: {err}')
    ok = moved > 5.0
    print(f'    -> {"MOVED" if ok else "DID NOT MOVE"}')
    print()
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=25.0,
                    help='seconds to wait for arm settle (default 25)')
    ap.add_argument('--hand-timeout', type=float, default=60.0,
                    help='seconds to tolerate for the hand call (default 60)')
    ap.add_argument('--skip-hand', action='store_true',
                    help='control run: steps 1 and 3 only, no hand call')
    args = ap.parse_args()

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except Exception as exc:  # noqa: BLE001
        print(f'FATAL: cannot use dummy_cli_tool ({exc})')
        return 2

    print('=== probe_can_wedge ===')
    print()
    print('  Question: does one hand call wedge the shared CAN bus, so that')
    print('  motor commands stop working afterwards?')
    print()
    print('  If YES, the driver must NEVER call hand.* -- not even to look.')
    print()

    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first.')
        return 1
    robot = dev.robot
    hand = robot.hand
    print('  connected')
    print()

    print('=' * 74)
    print('STEP 1 -- motor control before anything touches the gripper')
    print('=' * 74)
    # start from a known pose
    try:
        robot.move_j(*HOME)
    except Exception:  # noqa: BLE001
        pass
    wait_settle(robot, args.timeout)
    if not move_test(robot, ARM_A, 'move_j -> ARM_A (j1 -15)', args.timeout):
        print('  The arm did not move BEFORE any hand call, so this test cannot')
        print('  tell us anything about wedging. Fix motor control first.')
        return 1

    print('=' * 74)
    print('STEP 2 -- one hand call')
    print('=' * 74)
    if args.skip_hand:
        print('  --skip-hand: control run, not touching the hand')
        print('  Expect step 3 to MOVE. If it does, the test harness is sound.')
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
            print('    It returned. Note that it returned None more often than')
            print('    not: the client does not raise after exhausting its')
            print('    retries, it just falls through and returns.')
        print()

    print('=' * 74)
    print('STEP 3 -- can the motor still command, after the hand call?')
    print('=' * 74)
    # go back to home first, so a failure here is about the bus, not distance
    try:
        robot.move_j(*HOME)
    except Exception:  # noqa: BLE001
        pass
    wait_settle(robot, args.timeout)
    moved_again = move_test(robot, ARM_A, 'move_j -> ARM_A again (j1 -15)',
                            args.timeout)

    print('=' * 74)
    print('VERDICT')
    print('=' * 74)
    if args.skip_hand:
        if moved_again:
            print('  Control run: the arm moved both times, with no hand call.')
            print('  So the harness is sound -- a failure after a hand call')
            print('  would mean the hand call is what broke it.')
        else:
            print('  Control run FAILED: the arm stopped moving even without a')
            print('  hand call. Motor control is not reliable enough for this')
            print('  test; something else is wrong.')
        return 0 if moved_again else 1

    if moved_again:
        print('  OUTCOME A: the arm still moves after a hand call.')
        print()
        print('  The shared-CAN-wedge hypothesis is WRONG. The bus survives, so')
        print('  the fault is inside the hand path itself: either the frame is')
        print('  not reaching the servo, or the servo is not answering on it.')
        print()
        print('  Next: look at the servo, not the firmware.')
    else:
        print('  OUTCOME B: the arm STOPPED moving after one hand call.')
        print()
        print('  The shared-CAN-wedge hypothesis is CONFIRMED.')
        print()
        print('  One hand call permanently disables motor commands until the')
        print('  controller is power-cycled. That matches the firmware:')
        print('  CanSendMessage acquires sem_can1_tx with osWaitForever and')
        print('  never releases it, so if the TX-complete callback does not fire')
        print('  for the gripper frame, every later send blocks forever.')
        print()
        print('  Consequences, both important:')
        print('    1. the driver must NEVER call hand.* -- one call costs the')
        print('       whole arm until a power cycle')
        print('    2. this is a firmware bug with a clear report to the vendor:')
        print('       "set_angle wedges CAN TX for all subsequent commands"')
    return 0


if __name__ == '__main__':
    sys.exit(main())
