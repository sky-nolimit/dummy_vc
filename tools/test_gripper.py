#!/usr/bin/env python3
"""Drive the gripper and report what happened.

Why a script: the gripper has no position feedback beyond the raw device angle,
and the angle-to-opening mapping is not calibrated yet. This script sends a
normalised opening, then reports both the commanded value and the raw angle the
device reports back, so the mapping can be established from measurements.

Usage:
    python3 tools/test_gripper.py --open 1.0     # fully open
    python3 tools/test_gripper.py --open 0.0     # fully closed
    python3 tools/test_gripper.py --sweep        # step through and report
    python3 tools/test_gripper.py --hold 0.5 --seconds 3

SAFETY: start with --open 1.0 (opening moves away from the object). Closing
onto something is where damage happens. Set a low grip_current_limit first if
you are testing on a fragile object:
    ros2 launch dummy_controller dummy_controller.launch.py \
      grip_current_limit:=0.3
"""
import argparse
import os
import re
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'config', 'dummy_controller.yaml')
TOPIC = '/dummy_gripper/command'
STATE_TOPIC = '/dummy_gripper/state'


def get_scalar(text, key, default=None):
    m = re.search(rf'^\s*{key}\s*:\s*([-\d.]+)', text, re.M)
    return float(m.group(1)) if m else default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--open', type=float, default=None,
                    help='normalised opening: 0.0 closed, 1.0 open')
    ap.add_argument('--sweep', action='store_true',
                    help='step 0.0 -> 1.0 in 0.1 increments')
    ap.add_argument('--hold', type=float, default=None,
                    help='send this value and hold it')
    ap.add_argument('--seconds', type=float, default=2.0,
                    help='settle time per step (default 2)')
    args = ap.parse_args()

    if args.sweep:
        steps = [round(i * 0.1, 1) for i in range(11)]
    elif args.hold is not None:
        steps = [args.hold]
    elif args.open is not None:
        steps = [args.open]
    else:
        print('nothing to do: pass --open, --hold or --sweep')
        return 1

    text = open(CONFIG, encoding='utf-8').read()
    lo = get_scalar(text, 'grip_angle_closed', 0.0)
    hi = get_scalar(text, 'grip_angle_open', 90.0)
    ilim = get_scalar(text, 'grip_current_limit', 1.0)
    print(f'config: grip_angle_closed={lo} grip_angle_open={hi} '
          f'grip_current_limit={ilim}')
    print('  NOTE: grip_angle_closed/open are still placeholders until')
    print('        measured. The raw angle column below is what you measure')
    print('        against to fix them.')
    print()

    rclpy.init()
    node = Node('gripper_test')
    pub = node.create_publisher(Float64, TOPIC, 10)
    last_state = {'v': None}

    def cb(msg):
        last_state['v'] = msg.data

    node.create_subscription(Float64, STATE_TOPIC, cb, 10)

    # let discovery settle before the first command
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'  {"cmd frac":>9} {"implied angle":>14} {"state frac":>11}')
    for frac in steps:
        msg = Float64()
        msg.data = float(frac)
        for _ in range(5):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)

        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        implied = lo + (hi - lo) * frac
        state = last_state['v']
        state_s = 'n/a' if state is None else f'{state:>11.3f}'
        print(f'  {frac:>9.2f} {implied:>14.2f} {state_s}')

    print()
    print('How to use this to calibrate:')
    print('  1. Send --open 0.0, measure the actual gap between the fingers.')
    print('  2. Send --open 1.0, measure the gap again.')
    print('  3. If the fingers did not move, or moved the wrong way, the')
    print('     sign or the grip_angle_closed/open values are wrong.')
    print('  4. Once the mapping is known, put the measured angles into')
    print('     config/dummy_controller.yaml (grip_angle_closed / open).')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
