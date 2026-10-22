#!/usr/bin/env python3
"""Connecting to the device, and the steps that ACTUALLY energise it.

Why this module exists
----------------------
A probe that forgets a setup step does not fail loudly. It commands the device,
nothing happens, and the script reports "the arm did not move" -- which looks
exactly like the fault the probe was written to investigate. That has already
cost real time here: `probe_can_wedge.py` was written without
`robot.set_enable(True)` and concluded nothing, because an un-powered arm does
not move no matter what you send it.

So the setup lives in one place, and every probe calls it:

    connect()   find the device (with a timeout, never block forever)
    power_on()  enable the axes and set the RGB mode

What the driver does, for comparison
------------------------------------
`dummy_controller.py` on_configure:

    robot = ref_tool.find_any(path='usb', timeout=self.connect_timeout_sec)
    robot.set_rgb_mode(...)          # status indicator
    robot.set_enable(True)           # THIS is what applies torque

Without `set_enable(True)` the arm is limp: the device accepts commands and
does nothing with them. Note that `robot.set_enable(True)` also powers the
gripper's servo (measured: the fingers stiffen), so it is not only about the six
joints.

    from device_session import connect, power_on
    robot, hand = connect()
    power_on(robot)
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

# The RGB indicator mode the driver uses once it is up. Purely cosmetic, but
# setting it makes a probe's effect visible on the arm itself.
RGB_MODE_READY = 4

JR = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


def connect(timeout=15.0):
    """Find the device. Returns (robot, hand), or (None, None) with a message.

    Fibre's `find_any` blocks FOREVER when given no timeout, and with no device
    attached that becomes a silent hang with no output at all -- so the timeout
    is not optional here.

    A device that does not answer discovery RAISES TimeoutError rather than
    returning None (fibre/discovery.py -> utils.wait). That was learned the hard
    way: a probe reported a traceback instead of "no device" at exactly the
    moment the finding was "the board stopped answering", which read as a script
    bug rather than a measurement.
    """
    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except Exception as exc:  # noqa: BLE001
        print(f'FATAL: cannot use dummy_cli_tool ({exc})')
        return None, None
    try:
        dev = ref_tool.find_any(path='usb', timeout=timeout)
    except TimeoutError:
        print(f'FATAL: nothing answered discovery within {timeout:.0f}s.')
        print('       Either no device is attached, or the controller is hung')
        print('       and is not servicing USB at all. Power-cycle it and see')
        print('       whether discovery comes back.')
        print('       Stop dummy_controller first -- only one process can hold')
        print('       the fibre link.')
        return None, None
    if dev is None:
        print('FATAL: no device found.')
        print('       Stop dummy_controller first -- only one process can hold')
        print('       the fibre link.')
        return None, None
    return dev.robot, dev.robot.hand


def power_on(robot, rgb_mode=RGB_MODE_READY):
    """Enable the axes. Without this the arm does not move at all.

    Returns True if the call returned. A False here means the enable itself
    blocked, which is a finding in its own right -- do not proceed to command
    motion and then report "did not move".
    """
    try:
        robot.set_enable(True)
    except Exception as exc:  # noqa: BLE001
        print(f'  WARNING: robot.set_enable(True) raised: '
              f'{type(exc).__name__}: {exc}')
        return False
    try:
        robot.set_rgb_mode(rgb_mode)
    except Exception as exc:  # noqa: BLE001
        # Cosmetic only; a failure here does not affect motion.
        print(f'  note: set_rgb_mode({rgb_mode}) raised '
              f'{type(exc).__name__}; continuing')
    return True


def raw_angles(robot):
    """The six raw readback angles, in degrees."""
    return [float(getattr(robot, n).angle) for n in JR]


def wait_settle(robot, timeout=25.0, eps=0.05, quiet=0.8, poll=0.2):
    """Poll until the raw angles stop changing.

    Returns (angles, elapsed, truncated). A fixed sleep is wrong on this rig:
    the arm is slow and its start-up lag is a second or two, so a short sleep
    reads the starting pose and a long one wastes minutes.
    """
    t0 = time.monotonic()
    last = raw_angles(robot)
    stable = time.monotonic()
    while time.monotonic() - t0 < timeout:
        time.sleep(poll)
        now = raw_angles(robot)
        if max(abs(now[i] - last[i]) for i in range(6)) > eps:
            stable = time.monotonic()
        elif time.monotonic() - stable > quiet:
            return now, time.monotonic() - t0, False
        last = now
    return raw_angles(robot), time.monotonic() - t0, True


def move_and_report(robot, cmd, label, timeout=25.0, threshold=5.0,
                    index=0):
    """Send a move_j target and say whether the joint actually moved.

    `index` picks which joint to judge; the others can jog slightly when a new
    command arrives, so judging all six would hide a joint that never moved.
    """
    before = raw_angles(robot)
    err = None
    try:
        robot.move_j(*cmd)
    except Exception as exc:  # noqa: BLE001
        err = f'{type(exc).__name__}: {exc}'
    after, elapsed, trunc = wait_settle(robot, timeout)
    moved = abs(after[index] - before[index])
    ok = moved > threshold
    print(f'  {label}')
    print(f'    j{index + 1}: {before[index]:+9.3f} -> {after[index]:+9.3f}  '
          f'(moved {moved:6.3f} deg, {elapsed:4.1f}s'
          f'{"  WAIT TRUNCATED" if trunc else ""})')
    if err:
        print(f'    RAISED: {err}')
    print(f'    -> {"MOVED" if ok else "DID NOT MOVE"}')
    print()
    return ok
