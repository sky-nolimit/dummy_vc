"""阶段 0 标定的一键启动。

启动：robot_state_publisher + 标定节点 + RViz。
不启动 dummy_controller —— 避免两个进程同时抢串口设备。
标定结束后再启动 driver 做阶段 0.3 的对照。
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('dummy2_urdf_description')
    calib_share = get_package_share_directory('dummy_vc_calibration')

    xacro_file = os.path.join(desc_share, 'urdf', 'dummy2_urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    rviz_config = os.path.join(calib_share, 'rviz', 'stage0.rviz')

    # ---- launch 参数 ----
    declare_dry = DeclareLaunchArgument(
        'dry_run', default_value='false',
        description='true 则只读：拦截运动指令，但仍连接设备并回读。'
                    '注意 dry_run 不会跳过设备连接。')
    declare_path = DeclareLaunchArgument(
        'device_path', default_value='usb',
        description='设备发现路径，如 usb / serial:/dev/ttyUSB0')
    declare_timeout = DeclareLaunchArgument(
        'device_timeout', default_value='15.0',
        description='find_any 的超时秒数。设备被 dummy_controller 占用时会'
                    '一直等不到，可调小以便更快看到失败。')
    declare_handmax = DeclareLaunchArgument(
        'hand_max_angle', default_value='1.0',
        description='夹爪全开对应的设备角度，阶段 0.4 实测后修正')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='是否启动 RViz')
    declare_cur = DeclareLaunchArgument(
        'safe_current_limit', default_value='1.0',
        description='夹爪保守电流上限（启动时先设这个值）')

    return LaunchDescription([
        declare_dry,
        declare_path,
        declare_timeout,
        declare_handmax,
        declare_rviz,
        declare_cur,

        LogInfo(msg='阶段 0 标定：dry_run 只拦截运动指令，不影响连接与回读。'
                    '标定节点与 dummy_controller 不能同时持有 fibre 设备。'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='dummy_vc_calibration',
            executable='stage0_calibrate',
            name='stage0_calibrate',
            output='screen',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'device_timeout': LaunchConfiguration('device_timeout'),
                'dry_run': LaunchConfiguration('dry_run'),
                'hand_max_angle': LaunchConfiguration('hand_max_angle'),
                'safe_current_limit': LaunchConfiguration('safe_current_limit'),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
