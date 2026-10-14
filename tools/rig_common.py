#!/usr/bin/env python3
"""Shared helpers for the drive-side test scripts.

The one thing worth centralising here is waiting for the arm. **A fixed sleep is
wrong on this rig**, and it has caused real damage: an earlier probe reported
"device ignores single-joint commands" because its 0.8 s waits expired mid-travel
and every sample read the starting pose. That conclusion cost several rounds of
debugging before it was disproved.

Why "watch for a quiet window" is not enough on its own
-------------------------------------------------------
The device does not start moving the instant it receives a command. Every
measurement so far shows a start-up lag: probe_range records a settle time of
about 1.3 s even for a command whose target equals the current pose, i.e. one
where the arm does not move at all. So a short quiet window right after a command
proves nothing -- it may be the lag before motion, not the end of it. A version
of this module that returned on the first quiet window reported "joint did not
move" while the arm was still stationary and about to start, which is exactly the
false-failure mode it was written to prevent.

So waiting happens in two phases:

    phase 1  wait for the joints to START changing, up to `start_lag`
    phase 2  wait for them to STOP changing, up to `timeout`

If phase 1 expires with no movement at all, the caller is told `moved = False`.
That is genuine information: the device drops targets it will not act on, and it
also does nothing when the target equals the current pose, so the caller has to
decide which it was from how far it asked the joint to travel.

Measured speeds for sizing the timeouts (from probe_range):

    a 15 deg move settles in about 2 s
    a 150 deg move takes about 5 s
    j3 travelling 180 deg took about 18 s
"""
import math
import time

JOINTS = [f'dummy2_joint{i}' for i in range(1, 7)]
TRAJ_TOPIC = '/joint_trajectory_controller/joint_trajectory'

# a move counts as finished once nothing changed by more than this for QUIET_S
EPS_DEG = 0.05
QUIET_S = 1.0
# how long with no change at all before we conclude nothing is going to happen
START_LAG_S = 4.0


def angles_deg(state):
    """Joint angles from a /joint_states dict, in degrees."""
    return [math.degrees(state[j]) if j in state else float('nan')
            for j in JOINTS]


class WaitResult:
    """Outcome of waiting for the arm."""

    def __init__(self, angles, elapsed, moved, timed_out):
        self.angles = angles
        self.elapsed = elapsed
        self.moved = moved
        self.timed_out = timed_out

    def __iter__(self):
        # keep (angles, elapsed, truncated) unpacking working
        return iter((self.angles, self.elapsed, self.timed_out))


def wait_settled(spin, read, timeout=60.0, eps=EPS_DEG, quiet=QUIET_S,
                 start_lag=START_LAG_S):
    """Wait for the arm to move (if it is going to) and then stop.

    `spin(seconds)` pumps the ROS callbacks; `read()` returns joint angles in
    degrees.

    Returns a WaitResult. `moved` says whether the joints ever changed by more
    than `eps`; `timed_out` says whether `timeout` was reached while they were
    still changing, which makes the reading inconclusive rather than a fault.
    """
    t0 = time.monotonic()
    start = read()
    if any(v != v for v in start):
        # state not populated yet; fail loudly rather than compare NaN
        return WaitResult(start, 0.0, False, True)

    last = start
    moved = False
    stable_since = None

    while time.monotonic() - t0 < timeout:
        spin(0.15)
        now = read()
        if any(v != v for v in now):
            continue
        delta = max(abs(now[i] - last[i]) for i in range(6))

        if delta > eps:
            moved = True
            stable_since = None            # still moving, restart the window
        else:
            if stable_since is None:
                stable_since = time.monotonic()
            held = time.monotonic() - stable_since
            # Before anything has moved, require the longer start-up lag: a
            # brief quiet spell there is the device winding up, not the end.
            need = quiet if moved else start_lag
            if held > need:
                total = max(abs(now[i] - start[i]) for i in range(6))
                return WaitResult(now, time.monotonic() - t0,
                                  moved or total > eps, False)
        last = now

    end = read()
    total = max(abs(end[i] - start[i]) for i in range(6))
    return WaitResult(end, time.monotonic() - t0, moved or total > eps, True)
