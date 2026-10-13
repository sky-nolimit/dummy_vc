#!/usr/bin/env python3
"""Shared helpers for the drive-side test scripts.

The one thing worth centralising here is waiting for the arm. **A fixed sleep is
wrong on this rig**, and it has caused real damage: an earlier probe reported
"device ignores single-joint commands" because its 0.8 s waits expired mid-travel
and every sample read the starting pose. That conclusion cost several rounds of
debugging before it was disproved.

The arm is slow and its speed is not constant. Measured with probe_range:

    a 15 deg move settles in about 2 s
    a 150 deg move takes about 5 s
    j3 travelling 180 deg took about 18 s

so a timeout sized for a small move silently truncates a large one, and a fixed
sleep long enough for the worst case wastes minutes over a full sweep.

`wait_settled` polls the joint angles until they stop changing, with a timeout
only as a backstop, and reports whether it hit that backstop so truncation is
visible rather than silent.
"""
import math
import time

JOINTS = [f'dummy2_joint{i}' for i in range(1, 7)]
TRAJ_TOPIC = '/joint_trajectory_controller/joint_trajectory'

# a move counts as finished once nothing changed by more than this for QUIET_S
EPS_DEG = 0.05
QUIET_S = 0.6


def angles_deg(state):
    """Joint angles from a /joint_states dict, in degrees."""
    return [math.degrees(state[j]) if j in state else float('nan')
            for j in JOINTS]


def wait_settled(spin, read, timeout=40.0, eps=EPS_DEG, quiet=QUIET_S):
    """Poll until the joints stop moving.

    `spin(seconds)` must pump the ROS callbacks; `read()` must return the
    current angles in degrees. Returns (angles, elapsed, truncated).
    """
    t0 = time.monotonic()
    last = read()
    stable_since = time.monotonic()
    while time.monotonic() - t0 < timeout:
        spin(0.15)
        now = read()
        if any(v != v for v in now):          # NaN: state not populated yet
            continue
        if max(abs(now[i] - last[i]) for i in range(6)) > eps:
            stable_since = time.monotonic()
        elif time.monotonic() - stable_since > quiet:
            return now, time.monotonic() - t0, False
        last = now
    return read(), time.monotonic() - t0, True
