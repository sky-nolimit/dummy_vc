"""一条命令起"看 + 抓"：视觉（含相机、TF）+ 抓取规划/状态机。

它只是把两件事拼起来：

  1. `dummy_vc_perception/vision_demo.launch.py`（URDF/TF、相机、vision_node）
  2. `pick_place_node`（抓取规划 + 任务状态机）

**不含机械臂驱动、也不含 MoveIt** —— 按 docs/08 的做法，它们各开一个终端，
这样每层的日志和退出都各归各的。MoveIt 没起来时 `pick_place_node` 会报
"MoveIt 动作服务器没连上"（这是它该报的错，不是这个 launch 的问题）。

`execute:=true` 才会真的下发执行；默认只规划（`plan_only`），因为夹爪还挡着
（A2：`hand.*` 调用会让整机失去响应），能验到 APPROACH 为止。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SHARE = get_package_share_directory('dummy_vc_grasp')
DEFAULT_PARAMS = os.path.join(SHARE, 'config', 'pick_place.yaml')


def generate_launch_description():
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dummy_vc_perception'), 'launch',
            'vision_demo.launch.py'])),
        launch_arguments={
            'camera': LaunchConfiguration('camera'),
            'use_depth': LaunchConfiguration('use_depth'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('vision')),
    )

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=DEFAULT_PARAMS,
            description='抓取节点的参数文件'),
        DeclareLaunchArgument(
            'vision', default_value='true',
            description='是否顺带起视觉那一半（相机已经在跑就传 false）'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='传给视觉 launch：是否顺带起 Orbbec 相机'),
        DeclareLaunchArgument(
            'use_depth', default_value='false',
            description='传给视觉 launch：用深度算抓取点（这台相机上不可用）'),
        DeclareLaunchArgument(
            'execute', default_value='false',
            description='true 才真的执行规划结果；默认只规划'),
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description='true 则节点一启动就开始等目标（否则等 /pick_place/start）'),
        DeclareLaunchArgument(
            'place_x', default_value='0.0', description='放置点 x（base_link 下，米）'),
        DeclareLaunchArgument(
            'place_y', default_value='0.10', description='放置点 y（base_link 下，米）'),
    ]

    pick_place = Node(
        package='dummy_vc_grasp',
        executable='pick_place_node',
        name='pick_place_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'execute': ParameterValue(LaunchConfiguration('execute'),
                                          value_type=bool),
                'auto_start': ParameterValue(LaunchConfiguration('auto_start'),
                                             value_type=bool),
                'place_x': ParameterValue(LaunchConfiguration('place_x'),
                                          value_type=float),
                'place_y': ParameterValue(LaunchConfiguration('place_y'),
                                          value_type=float),
            },
        ],
    )

    return LaunchDescription(args + [vision, pick_place])
