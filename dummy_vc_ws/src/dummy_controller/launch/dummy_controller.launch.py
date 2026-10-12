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
   different value without editing the YAML and without rebuilding:

       ros2 launch dummy_controller dummy_controller.launch.py \\
         joint_sign:="[-1.0, 1.0, -1.0, 1.0, -1.0, -1.0]"

   This matters because ROS 2 launch does not reliably forward
   `--ros-args -p name:=value` to a node declared with parameters=[file].
   Explicit launch arguments are the dependable path.

Usage:
    ros2 launch dummy_controller dummy_controller.launch.py
    ros2 launch dummy_controller dummy_controller.launch.py activate:=false
    ros2 launch dummy_controller dummy_controller.launch.py \\
        joint_sign:="[-1.0, 1.0, -1.0, 1.0, -1.0, -1.0]"

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


# Defaults mirror config/dummy_controller.yaml. Keep the two in sync; the
# checker tools/selftest_conversion.py verifies that the YAML, the driver code
# defaults and these values all agree.
DEFAULTS = {
    'joint_sign': '[-1.0, 1.0, -1.0, 1.0, -1.0, -1.0]',
    'joint_offset_deg': '[0.215, 74.572, -91.474, -0.910, -0.035, -0.040]',
    'move_j_offset_deg': '[0.0, 0.0, 90.0, 0.0, 0.0, 0.0]',
    'urdf_offset_deg': '[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
    'home_joints_deg': '[0.0, 0.0, 90.0, 0.0, 0.0, 0.0]',
    'reset_joints_deg': '[0.0, -75.0, 180.0, 0.0, 0.0, 0.0]',
    # Gripper. Scalar values, not lists, so they are passed as plain strings.
    'grip_angle_closed': '0.0',
    'grip_angle_open': '90.0',
    'grip_current_limit': '1.0',
    'connect_timeout_sec': '30.0',
}

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
    'home_joints_deg':
        'The 7 pose, in move_j space (device degrees).',
    'reset_joints_deg':
        'The retracted pose, in move_j space (device degrees).',
    'grip_angle_closed':
        'Device angle (deg) at which the gripper is fully closed.',
    'grip_angle_open':
        'Device angle (deg) at which the gripper is fully open.',
    'grip_current_limit':
        'Gripper current limit. The only guard against crushing an object; '
        'start low.',
    'connect_timeout_sec':
        'Seconds to wait for the device. Must be > 0: a non-positive value '
        'makes fibre block forever instead of failing.',
}


def generate_launch_description():
    driver_share = get_package_share_directory('dummy_controller')
    desc_share = get_package_share_directory('dummy2_urdf_description')

    params_file = os.path.join(driver_share, 'config', 'dummy_controller.yaml')

    xacro_file = os.path.join(desc_share, 'urdf', 'dummy2_urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

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

    # One launch argument per calibration value, so any of them can be tried
    # without editing files or rebuilding.
    for key, default in DEFAULTS.items():
        actions.append(DeclareLaunchArgument(
            key, default_value=default, description=DESCRIPTIONS[key]))

    controller_params = [params_file]
    controller_params.append({
        key: LaunchConfiguration(key) for key in DEFAULTS
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
