#!/usr/bin/env python3
"""Read the current pose, add a delta to one joint, send it, report the result.

This is the workhorse behind tools/test_motion.sh. Doing the conversion here
rather than by hand matters: the driver juggles three coordinate spaces and a
mis-computed angle would be sent to real hardware.

The delta is applied in URDF space. The driver's own inverse mapping
(urdf_to_movej) converts it to the device convention, so this script does not
need to know about move_j or angle offsets at all.

Usage:
    python3 tools/send_joint_delta.py 1 20
    python3 tools/send_joint_delta.py 3 -10 --time 3
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JOINT_NAMES = [f'dummy2_joint{i}' for i in range(1, 7)]
TOPIC = '/joint_trajectory_controller/joint_trajectory'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('joint', type=int, help='joint number 1..6')
    ap.add_argument('delta', type=float, help='delta in degrees, URDF space')
    ap.add_argument('--time', type=float, default=3.0,
                    help='trajectory duration in seconds (default 3)')
    ap.add_argument('--timeout', type=float, default=8.0)
    args = ap.parse_args()

    if not 1 <= args.joint <= 6:
        print('joint must be 1..6')
        return 1

    rclpy.init()
    node = Node('send_joint_delta')
    pub = node.create_publisher(JointTrajectory, TOPIC, 10)

    got = {}

    def cb(msg):
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                got[n] = msg.position[i]

    node.create_subscription(JointState, '/joint_states', cb, 10)

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and len(got) < 6:
        rclpy.spin_once(node, timeout_sec=0.2)

    missing = [n for n in JOINT_NAMES if n not in got]
    if missing:
        print(f'could not read /joint_states for: {missing}')
        print('Is dummy_controller running and activated?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    current = [got[n] for n in JOINT_NAMES]
    target = list(current)
    target[args.joint - 1] += math.radians(args.delta)

    print('current URDF pose (deg) : '
          f'{[round(math.degrees(v), 2) for v in current]}')
    print(f'adding {args.delta:+.1f} deg to {JOINT_NAMES[args.joint - 1]}')
    print('target  URDF pose (deg) : '
          f'{[round(math.degrees(v), 2) for v in target]}')

    traj = JointTrajectory()
    traj.joint_names = list(JOINT_NAMES)
    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in target]
    pt.time_from_start.sec = int(args.time)
    pt.time_from_start.nanosec = int((args.time % 1) * 1e9)
    traj.points.append(pt)

    # Publish a few times: the driver subscribes in on_activate and a single
    # best-effort sample can be missed while discovery settles.
    for _ in range(5):
        pub.publish(traj)
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'sent to {TOPIC}')
    time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
