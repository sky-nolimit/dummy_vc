"""Bring up the Dummy arm driver together with robot_state_publisher.

Why this launch file exists:

1. The lifecycle node creates its subscriptions only in on_activate. Upstream
   has no launch file for that, so the node silently does nothing until you
   manually run two `ros2 lifecycle set` commands. Here the configure and
   activate transitions are emitted automatically.

2. robot_state_publisher needs the URDF to compute TF from /joint_states.
   Running it alongside the driver makes "does the model follow the real arm"
   a single step.

3. Every calibration value is exposed as a launch argument, so you can try a
   different value for one run without editing any file:

       ros2 launch dummy_controller dummy_controller.launch.py \\
         joint_sign:="[-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]"

   This matters because ROS 2 launch does not reliably forward
   `--ros-args -p name:=value` to a node declared with parameters=[file].
   Explicit launch arguments are the dependable path.

   The DEFAULT of every one of these arguments is read from the YAML when the
   launch description is built. Nothing is repeated here. That is deliberate:
   a second hardcoded copy of the calibration values is exactly what used to
   drift and then silently override the config, so editing the YAML appeared
   to do nothing. config/dummy_controller.yaml is the single source of truth;
   tools/selftest_conversion.py enforces that this file holds no second copy.

Usage:
    ros2 launch dummy_controller dummy_controller.launch.py
    ros2 launch dummy_controller dummy_controller.launch.py activate:=false
    ros2 launch dummy_controller dummy_controller.launch.py \\
        joint_sign:="[-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]"

About the lifecycle commands below
----------------------------------
They are issued by the `ros2` CLI, which needs the node to have registered
with DDS first. A single attempt can therefore fail with "Node not found" if it
runs too early, and it is NOT retried by anything. Two attempts per transition
are scheduled a few seconds apart; the first may fail harmlessly, the second
normally lands. The node itself also keeps working while unconfigured, so even
a missed command does not corrupt anything.

Do not be alarmed by a stray "Node not found" on a boot that nevertheless ends
with /joint_states publishing.
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


# Descriptions for the launch arguments. The VALUES are not here on purpose:
# they are read from config/dummy_controller.yaml by load_config_defaults().
DESCRIPTIONS = {
    'joint_sign':
        'Per-joint direction. Flip a joint to -1.0 if the model turns the '
        'opposite way from the real arm. Order: j1..j6.',
    'joint_offset_deg':
        'Raw device angle (deg) at which the model reads zero. Must equal the '
        'angle readback at the 7 pose.',
    'move_j_offset_deg':
        'Added to the URDF angle (deg) to get the move_j command value. '
        'Equal to the move_j value at the 7 pose.',
    'urdf_offset_deg':
        'Extra offset applied inside URDF space. Use only when the model '
        'turns the SAME way but is shifted by a constant angle.',
    'wrap_min_deg':
        'Lower bound the move_j command is normalised into.',
    'wrap_max_deg':
        'Upper bound the move_j command is normalised into. A span narrower '
        'than 360 deg makes the normaliser clamp instead of wrapping.',
    'max_move_jump_deg':
        'Largest travel (deg) a single move_j may ask for. Beyond this the '
        'command is refused instead of sent, because the device ignores '
        'unreachable setpoints silently.',
    'state_read_hz':
        'How often the device is polled for fresh angles (Hz). Device I/O is '
        'blocking; a slower read does not affect correctness.',
    'joint_state_pub_hz':
        'How often /joint_states is published from the cached angles (Hz). '
        'This is what makes RViz animate smoothly, and it does not touch the '
        'device.',
    'movej_urdf_zero_deg':
        'The move_j value that corresponds to URDF all-zeros (the 7 pose), '
        'per joint. This is the only constant in the down-link; it equals '
        'home_joints_deg.',
    'movej_accepted_range_deg':
        'Per-joint lo,hi of move_j values the device accepts (12 numbers). '
        'Values outside are dropped SILENTLY, taking the whole command with '
        'them. Measured with tools/probe_range.py.',
    'movej_current_trim_deg':
        'Per-joint trim (deg) added to the reference pose the driver keeps '
        'for whole-turn folding. Use it only to compensate a mis-anchored '
        'start pose; it does not change the pose homing() reaches.',
    'home_joints_deg':
        'The 7 pose, in move_j space (device degrees).',
    'reset_joints_deg':
        'The retracted pose, in move_j space (device degrees).',
    'finger_travel_m':
        'Travel of one finger (m), matching the URDF limit upper.',
    'grip_angle_closed':
        'Device angle (deg) at which the gripper is fully closed.',
    'grip_angle_open':
        'Device angle (deg) at which the gripper is fully open.',
    'grip_current_limit':
        'Gripper current limit. The only guard against crushing an object; '
        'start low.',
    'gripper_enable':
        'Initialise the gripper at startup. Default false: the gripper fibre '
        'calls have been observed to block forever, and the stuck thread '
        'holds the link so move_j starves and the arm cannot move.',
    'gripper_init_timeout_sec':
        'Seconds to wait for gripper init before giving up on it. The '
        'gripper fibre channel has blocked before, and a hang there must not '
        'freeze the whole node.',
    'connect_timeout_sec':
        'Seconds to wait for the device. Must be > 0: a non-positive value '
        'makes fibre block forever instead of failing.',
    'trajectory_action_enable':
        'Create the FollowJointTrajectory action server that MoveIt executes '
        'through. Without it planning still works but Execute fails with '
        '"Action client not connected". Turn it off only when debugging the '
        'driver without MoveIt.',
    'trajectory_action_name':
        'Name of that action server. Must equal /<controller_name>/<action_ns> '
        'from dummy_vc_moveit_config/config/moveit_controllers.yaml, i.e. '
        '/joint_trajectory_controller/follow_joint_trajectory. Do NOT append '
        'follow_joint_trajectory a second time: action_ns already IS that '
        'name, and both mistakes produce the same unhelpful error.',
    'trajectory_feedback_hz':
        'Rate (Hz) at which execution progress is logged to the driver log; '
        '0 or negative logs only the first, every tenth and the last '
        'waypoint. Feedback is NOT published on the action (MoveIt waits for '
        'the RESULT), so this does not affect whether Execute succeeds.',
}


def _as_launch_arg(value):
    """Render a YAML parameter value as the string a launch argument needs."""
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(repr(float(v)) for v in value) + ']'
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return repr(value)


def load_config_defaults(params_file):
    """Read every parameter default out of the YAML config.

    Returns {name: value-as-string}. Strings are what DeclareLaunchArgument
    wants, and rclpy parses '[...]' back into a double array for a parameter
    declared as one.

    Raising on a malformed config is intentional: a silent fallback to
    hardcoded defaults is what hid the drift this function exists to prevent.
    """
    with open(params_file, encoding='utf-8') as fh:
        doc = yaml.safe_load(fh)

    try:
        params = doc['dummy_controller']['ros__parameters']
    except (TypeError, KeyError) as exc:
        raise RuntimeError(
            f'{params_file}: expected a dummy_controller/ros__parameters '
            f'block, but parsing it failed ({exc})')

    if not params:
        raise RuntimeError(f'{params_file}: ros__parameters is empty')

    return {name: _as_launch_arg(value) for name, value in params.items()}


def generate_launch_description():
    driver_share = get_package_share_directory('dummy_controller')
    desc_share = get_package_share_directory('dummy2_urdf_description')

    params_file = os.path.join(driver_share, 'config', 'dummy_controller.yaml')

    xacro_file = os.path.join(desc_share, 'urdf', 'dummy2_urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Single source of truth for the calibration values.
    defaults = load_config_defaults(params_file)
    missing = sorted(set(defaults) - set(DESCRIPTIONS))
    if missing:
        raise RuntimeError(
            f'{params_file} declares parameters with no launch description: '
            f'{missing}. Add them to DESCRIPTIONS so they are documented and '
            'overridable.')

    # Two attempts per transition, spaced out. `ros2 lifecycle set` needs the
    # node registered with DDS, which takes a moment; the first attempt can
    # legitimately lose that race. The second normally lands.
    attempts = [(3.0, 8.0), (9.0, 14.0)]

    actions = [
        DeclareLaunchArgument(
            'activate', default_value='true',
            description='Emit configure then activate automatically'),
        DeclareLaunchArgument(
            'robot_state_publisher', default_value='true',
            description='Start robot_state_publisher (needed for TF in RViz)'),
    ]

    # One launch argument per YAML parameter, defaulting to the YAML value.
    # These exist purely so a value can be overridden for a single run.
    for key in sorted(defaults):
        actions.append(DeclareLaunchArgument(
            key, default_value=defaults[key], description=DESCRIPTIONS[key]))

    controller_params = [params_file]
    controller_params.append({
        key: LaunchConfiguration(key) for key in defaults
    })

    controller = LifecycleNode(
        package='dummy_controller',
        executable='dummy_controller',
        name='dummy_controller',
        namespace='',
        output='screen',
        parameters=controller_params,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(LaunchConfiguration('robot_state_publisher')),
    )

    actions += [
        LogInfo(msg='[bringup] starting dummy_controller (lifecycle node) ...'),
        controller,
        rsp,
    ]

    for n, (t_configure, t_activate) in enumerate(attempts, start=1):
        configure = ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/dummy_controller', 'configure'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('activate')),
        )
        activate = ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/dummy_controller', 'activate'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('activate')),
        )
        # Configure and activate each get their own timer slot so a slow
        # device connection (10-13 s) does not make activate overtake
        # configure. A rejected attempt is harmless: the node simply is not
        # ready yet, and the next attempt comes a few seconds later.
        actions.append(TimerAction(
            period=t_configure,
            actions=[
                LogInfo(msg=f'[bringup] attempt {n}: lifecycle configure ...'),
                configure,
            ],
        ))
        actions.append(TimerAction(
            period=t_activate,
            actions=[
                LogInfo(msg=f'[bringup] attempt {n}: lifecycle activate ...'),
                activate,
            ],
        ))

    actions.append(TimerAction(
        period=attempts[-1][1] + 1.0,
        actions=[LogInfo(msg=(
            '[bringup] lifecycle attempts finished. If /joint_states is '
            'publishing the driver is live; inspect the pose chain with: '
            'python3 tools/debug_pose_chain.py'))],
    ))

    return LaunchDescription(actions)
