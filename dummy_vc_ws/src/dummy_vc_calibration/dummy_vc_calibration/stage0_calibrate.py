#!/usr/bin/env python3
"""Stage 0 hardware calibration node for the Dummy arm.

Design: PREVIEW -> CONFIRM -> EXECUTE (two-phase commit).
Any motion that touches hardware first enters PREVIEW and is shown in RViz.
It is only sent to the device after an explicit /calib/confirm call.
This prevents a mistyped command from driving the arm into something.

NOTE: this file is deliberately ASCII-only. The workspace is edited on
Windows and built on Ubuntu; non-ASCII in source files has already caused
one silent encoding corruption. Chinese documentation lives in
docs/02-stage0-calibration-manual.md instead.

See docs/02-stage0-calibration-manual.md for the operating procedure.
"""
import glob
import math
import os
import sys
import threading
import time
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Make the vendored fibre library importable.
# dummy_ros2_driver is installed as an ament_python package, so its top level
# (which contains dummy_cli_tool/) ends up under install/<pkg>/lib/python3.x/
# site-packages. That directory is not on sys.path by default.
# ---------------------------------------------------------------------------
def _add_driver_to_path(logger=None):
    try:
        share = get_package_share_directory('dummy_controller')
    except Exception as exc:  # noqa: BLE001
        if logger:
            logger.error(f'cannot locate package dummy_controller: {exc}')
        return False

    prefix = os.path.dirname(os.path.dirname(share))  # install/dummy_controller
    libdir = os.path.join(prefix, 'lib')
    if not os.path.isdir(libdir):
        return False

    found = False
    for site in glob.glob(os.path.join(libdir, 'python3*', 'site-packages')):
        if site not in sys.path:
            sys.path.insert(0, site)
        found = True
    return found


_add_driver_to_path()

try:
    import dummy_cli_tool.ref_tool as ref_tool
    _HAVE_DRIVER = True
    _IMPORT_ERR = ''
except ImportError as _exc:  # noqa: BLE001
    ref_tool = None
    _HAVE_DRIVER = False
    _IMPORT_ERR = str(_exc)


# URDF joint names. Note the driver currently publishes joint_1..joint_6
# (a known mismatch, fixed in stage 1); the URDF is the source of truth.
URDF_JOINTS = [f'dummy2_joint{i}' for i in range(1, 7)]
FINGER_JOINTS = ['dummy2_finger_joint1', 'dummy2_finger_joint2']

# Prismatic finger travel limit, from the URDF.
FINGER_MAX = 0.023


class Phase:
    IDLE = 'IDLE'
    PREVIEW = 'PREVIEW'
    EXECUTING = 'EXECUTING'


class Stage0Calibrate(Node):

    # =======================================================================
    # Construction
    # =======================================================================
    def __init__(self):
        super().__init__('stage0_calibrate')

        # ---- parameters ----
        self.declare_parameter('device_path', 'usb')
        self.declare_parameter('device_timeout', 15.0)
        self.declare_parameter('send_period', 0.05)         # 20 Hz
        self.declare_parameter('max_joint_delta', 0.15)     # per-step guard
        self.declare_parameter('max_gripper_delta', 3.0)    # per-step guard
        self.declare_parameter('safe_current_limit', 1.0)   # conservative
        self.declare_parameter('dry_run', False)
        # Full-open gripper angle in device units. Replace after step 0.4.
        self.declare_parameter('hand_max_angle', 1.0)

        self.device_path = self.get_parameter('device_path').value
        self.device_timeout = self.get_parameter('device_timeout').value
        self.send_period = self.get_parameter('send_period').value
        self.max_joint_delta = self.get_parameter('max_joint_delta').value
        self.max_gripper_delta = self.get_parameter('max_gripper_delta').value
        self.safe_current_limit = self.get_parameter('safe_current_limit').value
        self.dry_run = self.get_parameter('dry_run').value

        # ---- state ----
        self.phase = Phase.IDLE
        self.dev = None
        self.robot = None
        self.hand = None

        self.joint_target = OrderedDict((j, 0.0) for j in URDF_JOINTS)
        self.joint_actual = OrderedDict((j, 0.0) for j in URDF_JOINTS)
        self.finger_pos = [0.0, 0.0]
        self.gripper_open_frac = 0.0
        self.gripper_angle = 0.0
        self.gripper_current_limit = 0.0
        self._last_driver_js = None

        self._lock = threading.Lock()

        # ---- publishers ----
        latching = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_ghost = self.create_publisher(
            JointState, '/calib/ghost_joint_states', latching)
        self.pub_marker = self.create_publisher(MarkerArray, '/calib/markers', 10)
        self.pub_target_pose = self.create_publisher(
            Pose, '/calib/target_pose', latching)

        # ---- services ----
        self.srv_joint = self.create_service(
            Trigger, '/calib/joint_step', self.on_joint_step)
        self.srv_gripper = self.create_service(
            Trigger, '/calib/gripper_step', self.on_gripper_step)
        self.srv_confirm = self.create_service(
            SetBool, '/calib/confirm', self.on_confirm)
        self.srv_snapshot = self.create_service(
            Trigger, '/calib/snapshot', self.on_snapshot)
        self.srv_safety = self.create_service(
            Trigger, '/calib/safety_check', self.on_safety_check)
        self.srv_compare = self.create_service(
            Trigger, '/calib/compare_driver', self.on_compare_driver)
        self.srv_timed = self.create_service(
            Trigger, '/calib/timed_move', self.on_timed_move)

        # ---- driver joint states (for the 0.3 cross-check) ----
        self.create_subscription(
            JointState, '/joint_states', self.on_driver_joint_state, 10)

        # ---- timer ----
        self.timer = self.create_timer(self.send_period, self.on_timer)

        # ---- connect ----
        #
        # IMPORTANT: dry_run only disables COMMANDING the hardware. It must
        # still connect and read back, otherwise read-only operations such as
        # /calib/compare_driver return a meaningless all-zero result.
        # (That bug made the 0.3 cross-check report driver=0 device=0 for every
        # joint, which looks like a valid measurement but is just no data.)
        self.get_logger().info('=' * 70)
        if not _HAVE_DRIVER:
            self.get_logger().error(f'cannot import dummy_cli_tool: {_IMPORT_ERR}')
            self.get_logger().error(
                'run colcon build and source install/setup.bash first')
        else:
            self.connect_device()

        # A loud, unmissable summary. The earlier version logged the
        # connection result inside connect_device only, so a failed connection
        # produced no obvious signal and the all-zero readback looked like a
        # real measurement.
        self.get_logger().info('=' * 70)
        if self.robot is None:
            self._warn_no_device_once()
            self.get_logger().error(
                'DEVICE: NOT CONNECTED -- readback will be all zeros')
        else:
            self.get_logger().info(
                f'DEVICE: CONNECTED (serial '
                f'{getattr(self.dev, "serial_number", "?")}) '
                f'-- readback is live')
        if self.dry_run:
            self.get_logger().warn(
                'MODE: dry_run=True -- motion commands blocked, '
                'readback and /calib/compare_driver work normally')
        else:
            self.get_logger().warn(
                'MODE: dry_run=False -- this node CAN move the hardware')
        self.get_logger().info('=' * 70)

        self.print_help()
        self.get_logger().info('=' * 70)

    # =======================================================================
    # Hardware
    # =======================================================================
    def connect_device(self):
        self.get_logger().info(f'looking for device (path={self.device_path}) ...')
        try:
            self.dev = ref_tool.find_any(
                path=self.device_path, timeout=self.device_timeout)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'find_any raised: {exc}')
            self.dev = None
            self.get_logger().error(
                'If dummy_controller is running it holds the fibre device. '
                'Stop it first, or start this node with dry_run:=true and use '
                'the driver-published /joint_states instead of direct readback.')
            return

        if self.dev is None:
            self.get_logger().error(
                'device not found. Check USB connection and udev rules.')
            self.get_logger().error(
                'If dummy_controller is already running, it holds the device. '
                'Only one process can own the fibre link at a time.')
            return

        self.robot = self.dev.robot
        self.hand = self.robot.hand
        self.get_logger().info(
            f'connected, serial = {getattr(self.dev, "serial_number", "?")}')

        # Conservative start: clamp gripper current BEFORE enabling it.
        # Skipped in dry_run: that mode must be strictly read-only.
        if self.dry_run:
            self.get_logger().info(
                'dry_run: skipping set_current_limit / set_enable / homing')
        else:
            try:
                self.hand.set_current_limit(self.safe_current_limit)
                self.gripper_current_limit = self.safe_current_limit
                self.get_logger().info(
                    f'gripper current limit set to {self.safe_current_limit}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'set_current_limit failed: {exc}')

            try:
                self.robot.set_enable(True)
                self.hand.set_enable(True)
                self.get_logger().info('joints and hand enabled')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'enable failed: {exc}')
                return

        # Seed the ghost target from the real pose so nothing jumps.
        # Works in dry_run too: this is a read, not a command.
        self.read_device()
        self.joint_target = OrderedDict(self.joint_actual)
        if self.robot is not None:
            self.get_logger().info('ghost target seeded from current pose')

    def read_device(self):
        """Read back device state. Never let an exception escape."""
        if self.robot is None:
            return
        try:
            for i, jname in enumerate(URDF_JOINTS):
                raw = float(getattr(self.robot, f'joint_{i + 1}').angle)
                self.joint_actual[jname] = raw
            self.gripper_angle = float(getattr(self.hand, 'angle', 0.0))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'readback failed: {exc}', throttle_duration_sec=5.0)

    def max_hand_angle(self):
        """Gripper full-open angle in device units."""
        return self.get_parameter('hand_max_angle').value

    # =======================================================================
    # Publishing
    # =======================================================================
    def publish_states(self):
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = list(self.joint_actual.keys()) + FINGER_JOINTS
        js.position = [self.joint_actual[j] for j in URDF_JOINTS] + list(self.finger_pos)
        self.pub_joint.publish(js)

        ghost = JointState()
        ghost.header.stamp = stamp
        ghost.name = list(self.joint_target.keys())
        ghost.position = [self.joint_target[j] for j in URDF_JOINTS]
        self.pub_ghost.publish(ghost)

        self.publish_markers(stamp)

    def publish_markers(self, stamp):
        arr = MarkerArray()

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'base_link'
        m.ns = 'calib_status'
        m.id = 0
        # ROS 2 does NOT have Marker.TEXT_VIEW_FAR_END (that is a ROS 1 name).
        # The valid constants are: ARROW, CUBE, SPHERE, CYLINDER, LINE_STRIP,
        # LINE_LIST, CUBE_LIST, SPHERE_LIST, POINTS, TEXT_VIEW_FACING, MESH,
        # TRIANGLE_LIST, INSTANCED_LINE_LIST.
        # TEXT_VIEW_FACING always faces the camera, which is what we want for
        # a status readout.
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.z = 0.65
        m.pose.orientation.w = 1.0
        m.scale.z = 0.03
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        lines = [f'phase = {self.phase}']
        if self.phase == Phase.PREVIEW:
            lines.append('PREVIEW pending - call /calib/confirm')
        lines.append('target: ' + str([round(self.joint_target[j], 4)
                                       for j in URDF_JOINTS]))
        lines.append('actual: ' + str([round(self.joint_actual[j], 4)
                                       for j in URDF_JOINTS]))
        lines.append(
            f'gripper: angle={self.gripper_angle:.3f} '
            f'frac={self.gripper_open_frac:.2f} '
            f'ilim={self.gripper_current_limit:.2f}')
        m.text = '\n'.join(lines)
        arr.markers.append(m)

        self.pub_marker.publish(arr)

    # =======================================================================
    # Safety gate
    # =======================================================================
    def _guard(self, what):
        if self.dry_run:
            self.get_logger().warn(f'[dry_run] blocked: {what}')
            return False
        if self.robot is None:
            self.get_logger().error(f'device not connected, refusing: {what}')
            return False
        return True

    def _clip(self, delta, limit, what):
        if abs(delta) > limit:
            self.get_logger().warn(
                f'{what} delta {delta:.4f} exceeds guard {limit}, clipped')
            return math.copysign(limit, delta)
        return delta

    @staticmethod
    def _parse_args(text):
        """Parse 'k=v,k=v'. std_srvs fields are too narrow, so we pack here."""
        out = {}
        for part in (text or '').replace(';', ',').split(','):
            part = part.strip()
            if not part or '=' not in part:
                continue
            k, v = part.split('=', 1)
            out[k.strip()] = v.strip()
        return out

    # =======================================================================
    # Services
    # =======================================================================
    def on_joint_step(self, request, response):
        """PREVIEW: joint N moves by delta (device units). Does not move yet."""
        args = self._parse_args(getattr(request, 'message', '') or '')
        try:
            idx = int(args.get('joint', 1))
            delta = float(args.get('delta', 0.0))
        except ValueError:
            response.success = False
            response.message = 'bad joint/delta argument'
            return response

        if not 1 <= idx <= 6:
            response.success = False
            response.message = 'joint must be 1..6'
            return response

        delta = self._clip(delta, self.max_joint_delta, 'joint')
        jname = URDF_JOINTS[idx - 1]

        with self._lock:
            newval = self.joint_target[jname] + delta
            self.joint_target[jname] = newval
            self.phase = Phase.PREVIEW
            actual = self.joint_actual[jname]

        response.success = True
        response.message = (
            f'PREVIEW joint{idx} ({jname}): target={newval:.4f} '
            f'actual={actual:.4f} delta={delta:+.4f} | confirm: '
            f'ros2 service call /calib/confirm std_srvs/srv/SetBool '
            f'"{{data: true}}"')
        self.get_logger().info(response.message)
        return response

    def on_gripper_step(self, request, response):
        """PREVIEW: gripper opening. Use open=<0..1> or delta=<d>."""
        args = self._parse_args(getattr(request, 'message', '') or '')
        with self._lock:
            if 'open' in args:
                try:
                    target = max(0.0, min(1.0, float(args['open'])))
                except ValueError:
                    response.success = False
                    response.message = 'bad open argument'
                    return response
                self.gripper_open_frac = target
            else:
                try:
                    delta = float(args.get('delta', 0.0))
                except ValueError:
                    response.success = False
                    response.message = 'bad delta argument'
                    return response
                delta = self._clip(delta, self.max_gripper_delta, 'gripper')
                self.gripper_open_frac = max(
                    0.0, min(1.0, self.gripper_open_frac + delta))
            self.phase = Phase.PREVIEW
            frac = self.gripper_open_frac

        response.success = True
        response.message = (
            f'PREVIEW gripper opening -> {frac:.3f} '
            f'(angle -> {frac * self.max_hand_angle():.4f}) | '
            f'confirm with /calib/confirm')
        self.get_logger().info(response.message)
        return response

    def on_confirm(self, request, response):
        """Execute the pending preview, or cancel it."""
        if not request.data:
            with self._lock:
                self.phase = Phase.IDLE
                self.joint_target = OrderedDict(self.joint_actual)
            response.success = True
            response.message = 'cancelled, ghost reset to actual pose'
            self.get_logger().info(response.message)
            return response

        if not self._guard('confirm/execute'):
            response.success = False
            response.message = 'device not connected or dry_run'
            return response

        with self._lock:
            if self.phase != Phase.PREVIEW:
                response.success = False
                response.message = f'phase is {self.phase}, nothing to confirm'
                return response
            targets = list(self.joint_target.values())
            frac = self.gripper_open_frac
            self.phase = Phase.EXECUTING

        self.get_logger().info(
            f'executing: joints={[round(v, 4) for v in targets]} '
            f'gripper_frac={frac:.3f}')
        self._do_execute(targets, frac)

        with self._lock:
            self.phase = Phase.IDLE
        response.success = True
        response.message = 'sent to device'
        return response

    def _do_execute(self, joint_targets, gripper_frac):
        try:
            hand_angle = gripper_frac * self.max_hand_angle()
            self.hand.set_angle(hand_angle)
            self.gripper_angle = hand_angle
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'gripper command failed: {exc}')

        for i, jname in enumerate(URDF_JOINTS):
            try:
                getattr(self.robot, f'joint_{i + 1}').set_position(joint_targets[i])
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'commanding {jname} failed: {exc}')

    def on_snapshot(self, request, response):
        """Log one line that can be pasted into docs/stage0_params.yaml."""
        rec = (
            'joints=[' + ', '.join(f'{self.joint_actual[j]:.6f}'
                                   for j in URDF_JOINTS) + '] '
            f'gripper_angle={self.gripper_angle:.6f} '
            f'gripper_frac={self.gripper_open_frac:.4f} '
            f'fingers=[{self.finger_pos[0]:.5f}, {self.finger_pos[1]:.5f}] '
            f'ilim={self.gripper_current_limit:.3f}'
        )
        self.get_logger().info(f'SNAPSHOT | {rec}')
        try:
            path = os.path.join(os.path.expanduser('~'), 'stage0_snapshots.log')
            with open(path, 'a', encoding='utf-8') as fh:
                fh.write(f'{time.time():.3f} | {rec}\n')
            response.message = f'logged, appended to {path}'
        except Exception as exc:  # noqa: BLE001
            response.message = f'logged (file append failed: {exc})'
        response.success = True
        return response

    def on_safety_check(self, request, response):
        """Read-only self test: connection, readback, temperatures."""
        lines = [f'phase={self.phase} dry_run={self.dry_run}',
                 f'driver_importable={_HAVE_DRIVER}',
                 f'device_connected={self.robot is not None}']
        ok = True

        if self.robot is not None:
            temps = []
            for i in range(1, 7):
                try:
                    t = getattr(self.robot, f'joint_{i}').get_temperature()
                    temps.append(f'j{i}={t:.1f}')
                except Exception:  # noqa: BLE001
                    temps.append(f'j{i}=?')
            lines.append('temps: ' + ' '.join(temps))
            lines.append('readback joints: '
                         + ', '.join(f'{self.joint_actual[j]:.4f}'
                                     for j in URDF_JOINTS))
            lines.append(f'readback hand: angle={self.gripper_angle:.4f} '
                         f'ilim={self.gripper_current_limit:.3f}')

        response.success = ok
        response.message = '\n'.join(lines)
        self.get_logger().info('SAFETY | ' + ' | '.join(lines))
        return response

    def on_driver_joint_state(self, msg):
        self._last_driver_js = msg

    def on_compare_driver(self, request, response):
        """Compare driver-published joint angles against raw device angles.

        This is how step 0.3 gets the URDF<->device sign and unit mapping.

        The arm must be at a NON-ZERO pose for this to be informative. If both
        columns read exactly 0 for every joint, either the arm really is at a
        singular zero pose, or (more likely) the device was not read at all:
        check the startup log for "connected, serial = ...".
        """
        if self._last_driver_js is None:
            response.success = False
            response.message = ('no /joint_states received yet. Start '
                                'dummy_controller first, then retry.')
            return response

        names = list(self._last_driver_js.name)
        pos = list(self._last_driver_js.position)

        device_ok = self.robot is not None
        lines = ['driver vs device raw:']
        if not device_ok:
            lines.append('  WARNING: this node did NOT connect to the device,')
            lines.append('  so the "device" column is all zeros and means')
            lines.append('  nothing. Check startup log for a connection line.')
        lines.append(f'  device_connected={device_ok}')

        all_zero = True
        for i, jname in enumerate(URDF_JOINTS):
            drv = None
            for cand in (jname, f'joint_{i + 1}'):
                if cand in names:
                    drv = pos[names.index(cand)]
                    break
            raw = self.joint_actual[jname]
            if abs(raw) > 1e-4 or (drv is not None and abs(drv) > 1e-4):
                all_zero = False

            if drv is None:
                lines.append(f'  {jname}: driver did not publish')
                continue

            diff = drv - raw
            note = ''
            # A difference near +/-2*pi means the two sides wrap differently.
            twopi = 2.0 * math.pi
            if device_ok and abs(abs(diff) - twopi) < 0.05:
                note = '  <-- looks like a 2*pi wrap difference'
            elif device_ok and abs(diff) > 1e-4:
                # if the ratio is close to -1, the sign is inverted
                if abs(raw) > 1e-3 and abs(drv + raw) < 0.05 * abs(raw):
                    note = '  <-- sign inverted (driver = -device)'
                else:
                    note = '  <-- non-zero offset'

            lines.append(
                f'  {jname}: driver={drv:+.6f} device={raw:+.6f} '
                f'diff={diff:+.6f}{note}')

        lines.append('')
        if all_zero and device_ok:
            lines.append('  All values are zero. This carries NO sign or')
            lines.append('  offset information. Move the arm to a clearly')
            lines.append('  non-zero pose (e.g. 30-45 degrees on several')
            lines.append('  joints) and call this again.')
        elif all_zero and not device_ok:
            lines.append('  All zeros AND the device was never read.')
            lines.append('  Stop dummy_controller so this node can own the')
            lines.append('  fibre link, then start with dry_run:=true.')

        response.success = True
        response.message = '\n'.join(lines)
        self.get_logger().info(' | '.join(lines))
        return response

    def on_timed_move(self, request, response):
        """Step 0.7: pin down the units of set_position_with_time(pos, time).

        Use: message='joint=1,pos=0.2,time=1.0'
        The node reports the wall-clock duration of the blocking call and the
        angle readback, so you can tell seconds-from-milliseconds and
        radians-from-degrees by observation.
        """
        if not self._guard('timed_move'):
            response.success = False
            response.message = 'device not connected or dry_run'
            return response

        args = self._parse_args(getattr(request, 'message', '') or '')
        try:
            idx = int(args.get('joint', 1))
            tgt = float(args.get('pos', 0.0))
            dur = float(args.get('time', 1.0))
        except ValueError:
            response.success = False
            response.message = 'bad joint/pos/time argument'
            return response

        if not 1 <= idx <= 6:
            response.success = False
            response.message = 'joint must be 1..6'
            return response

        before = self.joint_actual[URDF_JOINTS[idx - 1]]
        self.get_logger().warn(
            f'timed_move: joint{idx} pos={tgt} time={dur} (before={before:.4f})')

        t0 = time.monotonic()
        try:
            getattr(self.robot, f'joint_{idx}').set_position_with_time(tgt, dur)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'set_position_with_time raised: {exc}'
            return response
        call_secs = time.monotonic() - t0

        # Let the readback settle, then sample.
        time.sleep(max(dur, 0.5) + 0.3)
        self.read_device()
        after = self.joint_actual[URDF_JOINTS[idx - 1]]

        response.success = True
        response.message = (
            f'joint{idx}: pos {before:.4f} -> {after:.4f} '
            f'(requested {tgt}) | blocking_call={call_secs:.3f}s '
            f'requested_time={dur}')
        self.get_logger().info(response.message)
        return response

    # =======================================================================
    def on_timer(self):
        """Two independent jobs: read the device, then publish display data.

        They are separated on purpose. A mistake in the display code (for
        example an invalid Marker constant) must never stop the device
        readback, otherwise a cosmetic bug silently destroys all calibration
        data. That is exactly what happened once: a bad marker type made the
        timer throw every 50 ms and no device data was ever captured.
        """
        # --- job 1: device readback. Errors here are real problems. ---
        if self.robot is None:
            self._warn_no_device_once()
        else:
            try:
                self.read_device()
                half = FINGER_MAX * self.gripper_open_frac
                self.finger_pos = [half, half]
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f'device readback error: {exc}', throttle_duration_sec=5.0)

        # --- job 2: display. Errors here are cosmetic, warn but continue. ---
        try:
            self.publish_states()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'display publish error (device readback unaffected): {exc}',
                throttle_duration_sec=10.0)

    def _warn_no_device_once(self):
        if getattr(self, '_warned_no_device', False):
            return
        self._warned_no_device = True
        self.get_logger().error(
            'No device connection: all device readback is ZERO and means '
            'nothing. /calib/compare_driver will report device=+0.000000 for '
            'every joint. Look for "connected, serial = ..." earlier in this '
            'log; if it is absent the device was never opened. The usual cause '
            'is that dummy_controller is already running and holds the fibre '
            'link. Stop it, then restart this node.')

    def print_help(self):
        self.get_logger().info(
            '\nStage 0 calibration node ready. In another sourced terminal:\n'
            '\n  # 0.2/0.3 joint direction and units\n'
            '  ros2 service call /calib/joint_step std_srvs/srv/Trigger \\\n'
            '    "{message: \'joint=1,delta=0.3\'}"\n'
            '  ros2 service call /calib/confirm std_srvs/srv/SetBool "{data: true}"\n'
            '  ros2 service call /calib/confirm std_srvs/srv/SetBool "{data: false}"\n'
            '\n  # 0.4/0.5/0.6 gripper (0=closed, 1=open)\n'
            '  ros2 service call /calib/gripper_step std_srvs/srv/Trigger \\\n'
            '    "{message: \'open=0.5\'}"\n'
            '\n  # 0.7 units of set_position_with_time\n'
            '  ros2 service call /calib/timed_move std_srvs/srv/Trigger \\\n'
            '    "{message: \'joint=1,pos=0.2,time=1.0\'}"\n'
            '\n  # record a snapshot to ~/stage0_snapshots.log\n'
            '  ros2 service call /calib/snapshot std_srvs/srv/Trigger "{}"\n'
            '  # read-only self check\n'
            '  ros2 service call /calib/safety_check std_srvs/srv/Trigger "{}"\n'
            '  # driver angle vs device raw angle (needs dummy_controller running)\n'
            '  ros2 service call /calib/compare_driver std_srvs/srv/Trigger "{}"\n'
            '\nSee docs/02-stage0-calibration-manual.md')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Stage0Calibrate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # Re-clamp the gripper current before exiting so a large value
            # is not left latched on the device.
            try:
                if node.hand is not None:
                    node.hand.set_current_limit(node.safe_current_limit)
                    node.get_logger().info(
                        'exit: gripper current limit reset to safe value')
            except Exception:  # noqa: BLE001
                pass
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
