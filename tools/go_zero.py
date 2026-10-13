#!/usr/bin/env python3
"""Command the arm to the 7 pose (URDF all zeros) and report where it lands.

The 7 pose is the calibration anchor: `home_joints_deg` in move_j space maps to
URDF all zeros, and that is what the readback chain is built around. If the arm
cannot be driven back to URDF zero, the readback zero has drifted and every
published angle is offset by that much.

Success criterion: after settling, every joint in /joint_states is within
`--tol` degrees of zero.

**Waiting.** The arm is slow and its speed is not constant, so this polls until
the joints stop changing (`tools/rig_common.py`) rather than sleeping a fixed
time. A fixed sleep either truncates a long move -- reporting a false failure --
or wastes time on a short one. If the timeout is hit, that is said explicitly
and the check fails as inconclusive rather than as a fault.

Failure means the calibration anchor is wrong, not that the transport is broken
-- so the message says which.

    python3 tools/go_zero.py
    python3 tools/go_zero.py --tol 0.5 --timeout 90
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rig_common import JOINTS, TRAJ_TOPIC, angles_deg, wait_settled  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tol', type=float, default=1.5,
                    help='acceptance tolerance in degrees (default 1.5)')
    ap.add_argument('--timeout', type=float, default=60.0,
                    help='max seconds to wait for the move to settle (default '
                         '60). Raise it if the arm is slower than expected.')
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from builtin_interfaces.msg import Duration
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    except ImportError as exc:
        print(f'FAIL: cannot import ROS 2 python packages ({exc})')
        return 2

    rclpy.init()
    node = Node('go_zero')
    state = {}

    def on_state(msg):
        for name, pos in zip(msg.name, msg.position):
            state[name] = pos

    node.create_subscription(JointState, '/joint_states', on_state, 10)
    pub = node.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

    def spin(seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=0.05)

    def read():
        return angles_deg(state)

    # wait for a full joint state
    t0 = time.time()
    while time.time() - t0 < 6.0 and len(state) < 6:
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = [j for j in JOINTS if j not in state]
    if missing:
        print(f'FAIL: /joint_states is missing {missing}; is the driver up?')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    before, _, _ = wait_settled(spin, read, timeout=args.timeout)
    print(f'  before: {[round(v, 3) for v in before]} deg')

    traj = JointTrajectory()
    traj.joint_names = list(JOINTS)
    pt = JointTrajectoryPoint()
    pt.positions = [0.0] * 6
    pt.time_from_start = Duration(sec=3)
    traj.points = [pt]
    for _ in range(5):
        pub.publish(traj)
        rclpy.spin_once(node, timeout_sec=0.2)

    after, elapsed, truncated = wait_settled(spin, read, timeout=args.timeout)
    node.destroy_node()
    rclpy.shutdown()

    print(f'  after : {[round(v, 3) for v in after]} deg   '
          f'(settled in {elapsed:.1f}s)')
    if truncated:
        print(f'  RESULT: INCONCLUSIVE -- joints were still moving after '
              f'{args.timeout:.0f}s.')
        print('          Re-run with a larger --timeout; this is not a fault.')
        return 2
    worst = max(abs(v) for v in after if v == v)
    print(f'  worst |angle| = {worst:.3f} deg (tolerance {args.tol})')
    if worst <= args.tol:
        print('  RESULT: reached the 7 pose; the readback zero is consistent')
        return 0
    print('  RESULT: did NOT reach URDF zero.')
    print('          This is a calibration problem, not a transport one:')
    print('          the readback chain\'s zero (joint_offset_deg) does not')
    print('          match where the target chain actually parks the arm.')
    print('          The two chains use movej_urdf_zero_deg and')
    print('          joint_offset_deg separately, so check that they describe')
    print('          the same physical pose.')
    return 1


if __name__ == '__main__':
    sys.exit(main())

