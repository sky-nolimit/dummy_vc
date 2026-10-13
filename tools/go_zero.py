#!/usr/bin/env python3
"""Command the arm to the 7 pose (URDF all zeros) and report where it lands.

The 7 pose is the calibration anchor: `home_joints_deg` in move_j space maps to
URDF all zeros, and that is what the readback chain is built around. If the arm
cannot be driven back to URDF zero, the readback zero has drifted and every
published angle is offset by that much.

Success criterion: after settling, every joint in /joint_states is within
`--tol` degrees of zero.

Failure means the calibration anchor is wrong, not that the transport is broken
-- so the message says which.

    python3 tools/go_zero.py
    python3 tools/go_zero.py --tol 0.5
"""
import argparse
import math
import sys
import time

JOINTS = [f'dummy2_joint{i}' for i in range(1, 7)]
TOPIC = '/joint_trajectory_controller/joint_trajectory'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tol', type=float, default=1.5,
                    help='acceptance tolerance in degrees (default 1.5)')
    ap.add_argument('--settle', type=float, default=12.0,
                    help='seconds to wait after commanding')
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
    got = {}

    def on_state(msg):
        for name, pos in zip(msg.name, msg.position):
            got[name] = pos

    node.create_subscription(JointState, '/joint_states', on_state, 10)
    pub = node.create_publisher(JointTrajectory, TOPIC, 10)

    # wait for a full joint state
    t0 = time.time()
    while time.time() - t0 < 6.0 and len(got) < 6:
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = [j for j in JOINTS if j not in got]
    if missing:
        print(f'FAIL: /joint_states is missing {missing}; is the driver up?')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    before = [math.degrees(got[j]) for j in JOINTS]
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

    t0 = time.time()
    while time.time() - t0 < args.settle:
        rclpy.spin_once(node, timeout_sec=0.1)

    after = [math.degrees(got[j]) for j in JOINTS]
    node.destroy_node()
    rclpy.shutdown()

    print(f'  after : {[round(v, 3) for v in after]} deg')
    worst = max(abs(v) for v in after)
    print(f'  worst |angle| = {worst:.3f} deg (tolerance {args.tol})')
    if worst <= args.tol:
        print('  RESULT: reached the 7 pose; the readback zero is consistent')
        return 0
    print('  RESULT: did NOT reach URDF zero.')
    print('          This is a calibration problem, not a transport one:')
    print('          the readback chain\'s zero (joint_offset_deg) does not')
    print('          match where the target chain actually parks the arm.')
    print('          The two chains use the same joint_offset_deg, so if they')
    print('          disagree, one of the formulas or the offset sign is off.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
