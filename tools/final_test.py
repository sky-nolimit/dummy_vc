#!/usr/bin/env python3
"""Drive-side final acceptance test.

Every check states its success criterion and what a failure means. Nothing
passes silently. All commands are issued in URDF space, so the test does not
need to know any device offset -- if it needed to, the calibration would not be
testable from outside.

    # driver already running:
    python3 tools/final_test.py

    # skip the moving parts (safe, instantaneous):
    python3 tools/final_test.py --no-move

Checks
  1  /joint_states exists and is published at a usable rate
  2  the arm is stationary (angles do not drift on their own)
  3  each joint moves in BOTH directions when told to, and comes back
  4  the 7 pose is reachable: commanding URDF zero lands at URDF zero

**Timing.** This arm is slow and its speed is not constant, so every wait here
polls until the joints actually stop changing (tools/rig_common.py) instead of
sleeping a fixed time. A fixed sleep long enough for the worst case wastes
minutes, and one sized for a small move truncates a large one and reports a
false failure -- an earlier probe did exactly that and produced a completely
wrong conclusion. Each move prints how long it actually took and whether the
timeout was hit, so truncation is visible rather than silent.

Exit status 0 only if every check passes.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rig_common import JOINTS, TRAJ_TOPIC, angles_deg, wait_settled  # noqa: E402


class Result:
    def __init__(self):
        self.rows = []

    def add(self, ok, label, detail=''):
        self.rows.append((ok, label, detail))
        mark = 'PASS' if ok else 'FAIL'
        print(f'  [{mark}] {label}' + (f'   -- {detail}' if detail else ''))

    def skip(self, label, detail=''):
        self.rows.append((None, label, detail))
        print(f'  [SKIP] {label}' + (f'   -- {detail}' if detail else ''))

    @property
    def failures(self):
        return [r for r in self.rows if r[0] is False]


class Rig:
    """Minimal ROS 2 client for the checks below."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        self.rclpy = rclpy
        self.JointTrajectory = JointTrajectory
        self.JointTrajectoryPoint = JointTrajectoryPoint
        self.Duration = Duration

        rclpy.init()
        self.node = Node('final_test')
        self.state = {}
        self.stamps = []
        self.node.create_subscription(JointState, '/joint_states',
                                      self._on_state, 10)
        self.pub = self.node.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

    def _on_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.state[name] = pos
        self.stamps.append(time.monotonic())

    def spin(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def read(self):
        return angles_deg(self.state)

    def wait(self, timeout):
        return wait_settled(self.spin, self.read, timeout=timeout)

    def wait_for_state(self, timeout=8.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and len(self.state) < 6:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return [j for j in JOINTS if j not in self.state]

    def send(self, positions_rad, seconds=3):
        traj = self.JointTrajectory()
        traj.joint_names = list(JOINTS)
        pt = self.JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions_rad]
        pt.time_from_start = self.Duration(sec=int(seconds))
        traj.points = [pt]
        for _ in range(5):
            self.pub.publish(traj)
            self.rclpy.spin_once(self.node, timeout_sec=0.2)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def measure_rate(rig, seconds=5.0):
    rig.stamps.clear()
    rig.spin(seconds)
    n = len(rig.stamps)
    if n < 2:
        return None
    span = rig.stamps[-1] - rig.stamps[0]
    return (n - 1) / span if span > 0 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-move', action='store_true',
                    help='skip checks that move the arm')
    ap.add_argument('--tol-deg', type=float, default=1.5,
                    help='tolerance for "reached URDF zero" (default 1.5)')
    ap.add_argument('--min-hz', type=float, default=20.0,
                    help='minimum acceptable /joint_states rate (default 20)')
    ap.add_argument('--move-deg', type=float, default=10.0,
                    help='per-joint test amplitude (default 10)')
    ap.add_argument('--move-timeout', type=float, default=40.0,
                    help='max seconds to wait for one move to settle (default '
                         '40). Raise it if the arm is slower than expected; a '
                         'truncated wait is reported, never hidden.')
    args = ap.parse_args()

    res = Result()
    try:
        rig = Rig()
    except Exception as exc:  # noqa: BLE001
        print(f'FATAL: cannot start the ROS 2 client ({exc})')
        print('       Is the driver running, and is ROS sourced?')
        return 2

    print('=' * 70)
    print('1. /joint_states present and publishing')
    print('=' * 70)
    missing = rig.wait_for_state()
    if missing:
        res.add(False, '/joint_states carries all six joints',
                f'missing {missing}')
        rig.close()
        print('\nDriver not usable; stopping here.')
        return 1
    res.add(True, '/joint_states carries all six joints')

    hz = measure_rate(rig)
    if hz is None:
        res.skip('/joint_states rate', 'no messages seen')
    else:
        res.add(hz >= args.min_hz,
                f'/joint_states rate {hz:.1f} Hz >= {args.min_hz:.0f}',
                'publishing is decoupled from device reads; a low rate means '
                'something is blocking the executor again'
                if hz < args.min_hz else '')

    print()
    print('=' * 70)
    print('2. the arm is stationary when it should be')
    print('=' * 70)
    before, _, _ = rig.wait(5.0)
    print(f'  angles now: {[round(v, 3) for v in before]} deg')
    rig.spin(1.5)
    after = rig.read()
    print(f'  1.5 s later: {[round(v, 3) for v in after]} deg')
    drift = max(abs(after[i] - before[i]) for i in range(6))
    print(f'  worst drift = {drift:.4f} deg')
    res.add(drift < 2.0, 'no drift while idle',
            '' if drift < 2.0 else
            'angles change on their own: the readback is unstable, or the '
            'arm is being pushed')

    if args.no_move:
        print()
        print('  (--no-move: skipping the moving checks)')
        res.skip('per-joint direction test', '--no-move')
        res.skip('7 pose reachable', '--no-move')
        return 0 if not res.failures else 1

    print()
    print('=' * 70)
    print(f'3. each joint moves both ways by {args.move_deg:.0f} deg')
    print('=' * 70)
    print('  Success: a positive command moves the joint, the equal negative')
    print('  command brings it back. Failure means the joint does not move,')
    print('  the direction is inverted, or the command is dropped silently.')
    print('  Each move is waited on until the joints stop changing; the')
    print('  elapsed time is printed so a timeout-driven false reading is')
    print('  obvious.')
    truncated_any = False
    for idx, name in enumerate(JOINTS):
        # start from a settled pose so d1 is measured from rest
        a0, _, _ = rig.wait(args.move_timeout)

        target = [rig.state[j] for j in JOINTS]
        target[idx] = rig.state[name] + math.radians(args.move_deg)
        rig.send(target)
        a1, t1, trunc1 = rig.wait(args.move_timeout)
        truncated_any = truncated_any or trunc1

        target[idx] = rig.state[name] - math.radians(args.move_deg)
        rig.send(target)
        a2, t2, trunc2 = rig.wait(args.move_timeout)
        truncated_any = truncated_any or trunc2

        d1 = a1[idx] - a0[idx]
        d2 = a2[idx] - a1[idx]
        moved = abs(d1) > args.move_deg * 0.3
        returned = abs(d2) > args.move_deg * 0.3 and d1 * d2 < 0
        note = ''
        if trunc1 or trunc2:
            note = '  <-- WAIT TIMED OUT, readings may be mid-travel'
        print(f'  j{idx + 1}: {a0[idx]:9.3f} -> {a1[idx]:9.3f} -> '
              f'{a2[idx]:9.3f}   out {d1:+8.3f} ({t1:4.1f}s)  '
              f'back {d2:+8.3f} ({t2:4.1f}s){note}')
        if not moved:
            res.add(False, f'j{idx + 1} responds to a command',
                    f'commanded {args.move_deg:+.0f} deg, the joint moved '
                    f'{d1:+.3f}; the device drops targets it will not act on, '
                    'so check the accepted range and the calibration')
        elif not returned:
            res.add(False, f'j{idx + 1} returns when told',
                    f'out {d1:+.3f} then {d2:+.3f}: the direction is inverted '
                    'or the second command was ignored (joint_sign?)')
        else:
            res.add(True, f'j{idx + 1} moves both ways')

    print()
    print('=' * 70)
    print('4. the 7 pose is reachable (command URDF zero, land at zero)')
    print('=' * 70)
    print('  This is the calibration anchor. Missing it means the readback')
    print('  zero and the target zero disagree -- a calibration fault, not a')
    print('  transport one.')
    rig.send([0.0] * 6)
    landed, elapsed, trunc = rig.wait(args.move_timeout * 2)
    print(f'  landed at: {[round(v, 3) for v in landed]} deg   '
          f'(settled in {elapsed:.1f}s)')
    if trunc:
        res.add(False, 'returning to the 7 pose settles',
                f'joints were still moving after {args.move_timeout * 2:.0f}s; '
                're-run with a larger --move-timeout')
    else:
        worst = max(abs(v) for v in landed if v == v)
        print(f'  worst |angle| = {worst:.3f} deg '
              f'(tolerance {args.tol_deg})')
        res.add(worst <= args.tol_deg, 'URDF zero reaches URDF zero',
                '' if worst <= args.tol_deg else
                'the target chain and the readback chain disagree about where '
                'zero is; check movej_urdf_zero_deg and the down-link')

    rig.close()

    print()
    print('=' * 70)
    print('CONCLUSION')
    print('=' * 70)
    passed = sum(1 for ok, _, _ in res.rows if ok is True)
    skipped = sum(1 for ok, _, _ in res.rows if ok is None)
    print(f'  passed {passed}, failed {len(res.failures)}, skipped {skipped}')
    if truncated_any:
        print('\n  NOTE: at least one move hit its timeout, so a "did not')
        print('  move" result above may be a measurement artefact rather than')
        print('  a real fault. Re-run with a larger --move-timeout first.')
    if not res.failures:
        print('\n  ALL CHECKS PASSED -- the drive path is usable.')
        if skipped:
            print('  (some checks were skipped; see above)')
        return 0
    print('\n  FAILURES:')
    for _, label, detail in res.failures:
        print(f'    - {label}')
        if detail:
            print(f'        {detail}')
    return 1


if __name__ == '__main__':
    sys.exit(main())
