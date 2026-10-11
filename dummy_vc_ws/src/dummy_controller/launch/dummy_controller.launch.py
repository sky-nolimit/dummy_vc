"""Bring up the Dummy arm driver together with robot_state_publisher.

This launch file exists for two reasons:

1. The lifecycle node creates its subscriptions only in on_activate. Upstream
   has no launch file for that, so the node silently does nothing until you
   manually run two `ros2 lifecycle set` commands. Here the configure and
   activate transitions are emitted automatically.

2. robot_state_publisher needs the URDF to compute TF from /joint_states.
   Running it alongside the driver in one command makes the "does the model
   follow the real arm" test a single step.

Usage:
    ros2 launch dummy_controller dummy_controller.launch.py
    ros2 launch dummy_controller dummy_controller.launch.py activate:=false
    ros2 launch dummy_controller dummy_controller.launch.py robot_state_publisher:=false
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

    declare_activate = DeclareLaunchArgument(
        'activate', default_value='true',
        description='Emit configure then activate automatically')
    declare_rsp = DeclareLaunchArgument(
        'robot_state_publisher', default_value='true',
        description='Start robot_state_publisher (needed for TF in RViz)')

    controller = LifecycleNode(
        package='dummy_controller',
        executable='dummy_controller',
        name='dummy_controller',
        namespace='',
        output='screen',
        parameters=[params_file],
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

    actions = [
        declare_activate,
        declare_rsp,
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
                LogInfo(msg='[bringup] if you see /joint_states below, '
                            'the driver is live.'),
            ],
        ),
    ]
    return LaunchDescription(actions)
