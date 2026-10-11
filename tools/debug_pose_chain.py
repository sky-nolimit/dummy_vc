#!/usr/bin/env python3
"""Print the full pose chain, and say which pose the MODEL should be showing.

This exists because "the model looks wrong" is hard to act on: it does not say
whether the fault is in the raw device angle, the offset, the sign, or the
URDF model itself. This script prints every stage of the chain in one place and
then classifies the resulting model pose geometrically (using forward
kinematics on the real URDF), so the report says plainly:

    the model is showing: EXTENDED / RAISED / RETRACTED / DOWN

Run it with the driver up, or with the calibration node in dry_run read-only.

Usage:
    ros2 run ...              # not needed; run directly
    python3 tools/debug_pose_chain.py
    python3 tools/debug_pose_chain.py --timeout 5

Read-only: subscribes to /joint_states only.
"""
import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'config', 'dummy_controller.yaml')
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')

JOINTS = [f'dummy2_joint{i}' for i in range(1, 7)]


def get_list(text, key, default=None):
    m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
    if not m:
        return default
    return [float(v) for v in m.group(1).split(',') if v.strip()]


def classify(urdf_rad):
    """Describe the model pose from the joint angles, via real FK."""
    try:
        import fk_dummy
    except ImportError:
        return 'unknown (fk_dummy.py not importable)'

    links, joints = fk_dummy.parse_model(XACRO)
    rev = sorted([j for j in joints if j['type'] in ('revolute', 'continuous')],
                 key=lambda j: j['name'])
    names = [j['name'] for j in rev]
    angles = {names[i]: urdf_rad[i] for i in range(min(6, len(names)))}
    pts, T = fk_dummy.fk(joints, 'gripper_tcp', angles)

    x, y, z = T[0][3], T[1][3], T[2][3]
    reach = math.hypot(x, y)
    j3 = math.degrees(urdf_rad[2])

    print(f'  gripper_tcp = ({x:+.3f}, {y:+.3f}, {z:+.3f})')
    print(f'  reach = {reach:.3f} m   height = {z:+.3f} m')
    print(f'  j3 = {j3:+.1f} deg')

    # URDF zero has the arm extending along +y at z ~ 0.35, i.e. the 7 pose.
    # Measured TCP for reference poses:
    #   j3=0    (+0.012, +0.325, +0.346)  extended forward  <- 7 pose
    #   j3=90   (+0.012, -0.028, +0.574)  raised straight up
    #   j3=180  (+0.012, -0.256, +0.221)  folded backward over the base
    #   j3=-90  (+0.012, +0.097, -0.007)  folded down to base level
    # Order matters: check the distinctly-shaped poses first.
    if z < 0.10:
        shape = 'RETRACTED / folded down to base level'
    elif z > 0.50:
        shape = 'RAISED straight up'
    elif y < -0.15:
        shape = 'folded BACKWARD over the base (not the 7 pose)'
    elif reach > 0.25 and z > 0.30:
        shape = 'EXTENDED forward  <-- this is the 7 pose'
    else:
        shape = 'intermediate / folded'
    return shape


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=10.0)
    args = ap.parse_args()

    text = open(CONFIG, encoding='utf-8').read()
    sign = get_list(text, 'joint_sign')
    off = get_list(text, 'joint_offset_deg')
    mjoff = get_list(text, 'move_j_offset_deg')
    uoff = get_list(text, 'urdf_offset_deg')

    print('config offsets')
    print(f'  joint_sign         = {sign}')
    print(f'  joint_offset_deg   = {off}')
    print(f'  move_j_offset_deg  = {mjoff}')
    print(f'  urdf_offset_deg    = {uoff}')
    print()

    try:
        import rclpy
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        print(f'ROS 2 Python not available: {exc}')
        print('Start the driver, then run this again inside the ROS environment.')
        return 0

    rclpy.init()
    node = rclpy.create_node('pose_chain_debug')
    got = {}

    def cb(msg):
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                got[n] = msg.position[i]

    node.create_subscription(JointState, '/joint_states', cb, 10)

    import time
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and not got:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not got:
        print(f'no /joint_states within {args.timeout:.0f}s.')
        print('Is dummy_controller running and activated?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print('/joint_states as published (URDF space, radians)')
    urdf_rad = []
    for n in JOINTS:
        v = got.get(n)
        if v is None:
            print(f'  {n:<16} MISSING')
            urdf_rad.append(0.0)
        else:
            urdf_rad.append(v)
            print(f'  {n:<16} {v:+.5f} rad  ({math.degrees(v):+8.2f} deg)')
    print()

    # Reconstruct the upstream stages for cross-checking
    print('reconstructed chain (for cross-checking only)')
    urdf_deg = [math.degrees(v) for v in urdf_rad]
    chain = [urdf_deg[i] - uoff[i] for i in range(6)]
    ang = []
    for i in range(6):
        s = sign[i] if abs(sign[i]) > 1e-12 else 1.0
        ang.append(chain[i] / s + off[i])
    movej = [ang[i] + mjoff[i] for i in range(6)]
    print(f'  implied angle space  = {[round(v, 2) for v in ang]}')
    print(f'  implied move_j space = {[round(v, 2) for v in movej]}')
    print('  (compare the move_j line with home_joints_deg / '
          'reset_joints_deg in the config)')
    print()

    print('what the MODEL is showing right now:')
    shape = classify(urdf_rad)
    print(f'  => {shape}')
    print()

    cfg_home = get_list(text, 'home_joints_deg')
    cfg_reset = get_list(text, 'reset_joints_deg')
    print('reference: the config expects')
    print(f'  home  (7 pose)  move_j = {cfg_home}')
    print(f'  reset (retract) move_j = {cfg_reset}')
    print()
    print('If the reported shape disagrees with the real arm, note whether the')
    print('model moves in the SAME direction as the real arm when you push a')
    print('joint by hand. Same direction = offset problem (urdf_offset_deg).')
    print('Opposite direction = sign problem (joint_sign).')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
