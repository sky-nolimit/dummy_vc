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
         joint_offset_deg:="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

   This matters because ROS 2 launch does not reliably forward
   `--ros-args -p name:=value` to a node declared with parameters=[file].
   Explicit launch arguments are the dependable path.

Usage:
    ros2 launch dummy_controller dummy_controller.launch.py
    ros2 launch dummy_controller dummy_controller.launch.py activate:=false
    ros2 launch dummy_controller dummy_controller.launch.py \\
        joint_offset_deg:="[0.03, 2.03, 0.0, -0.03, 0.18, -0.03]"
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
# checker tools/check_coord_spaces.py verifies that the YAML, the driver code
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
    'connect_timeout_sec': '15.0',
}

DESCRIPTIONS = {
    'joint_sign':
        'Per-joint direction. Flip a joint to -1.0 if the model turns the '
        'opposite way from the real arm. Order: j1..j6.',
    'joint_offset_deg':
        'Per-joint offset between the device ANGLE readback and the URDF '
        'angle, in degrees. Derived from the 7-pose readback.',
    'move_j_offset_deg':
        'Per-joint offset between the device move_j space (what home/reset '
        'use) and the ANGLE readback, in degrees.',
    'urdf_offset_deg':
        'Extra offset applied in URDF space, in degrees. Use this when the '
        'model tracks the same direction but sits at a fixed wrong angle.',
    'home_joints_deg':
        'Named pose in move_j space (degrees). 7 pose.',
    'reset_joints_deg':
        'Named pose in move_j space (degrees). Retracted pose.',
    'grip_angle_closed':
        'Gripper closed angle in device units (degrees).',
    'grip_angle_open':
        'Gripper open angle in device units (degrees).',
    'grip_current_limit':
        'Gripper current limit (A) used when driving the gripper.',
}


def generate_launch_description():
    driver_share = get_package_share_directory('dummy_controller')
    desc_share = get_package_share_directory('dummy2_urdf_description')

    params_file = os.path.join(driver_share, 'config', 'dummy_controller.yaml')

    xacro_file = os.path.join(desc_share, 'urdf', 'dummy2_urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # The node creates its subscriptions in on_activate, so give it a moment
    # to finish configuring. 3 s is generous but harmless.
    delay_configure = 3.0
    delay_activate = 2.0

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

    # Transition the lifecycle node via the ros2 CLI. Using ExecuteProcess
    # avoids depending on lifecycle_msgs event APIs in launch.
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

    actions += [
        LogInfo(msg='[bringup] starting dummy_controller (lifecycle node) ...'),
        controller,
        rsp,
        TimerAction(
            period=delay_configure,
            actions=[
                LogInfo(msg='[bringup] emitting lifecycle configure ...'),
                configure,
            ],
        ),
        TimerAction(
            period=delay_configure + delay_activate,
            actions=[
                LogInfo(msg='[bringup] emitting lifecycle activate ...'),
                activate,
                LogInfo(msg='[bringup] if /joint_states appears below, '
                            'the driver is live. To inspect the pose chain '
                            'run: python3 tools/debug_pose_chain.py'),
            ],
        ),
    ]
    return LaunchDescription(actions)
