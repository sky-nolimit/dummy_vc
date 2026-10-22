"""一条命令起"看 + 抓"：视觉（含相机、TF）+ 抓取规划/状态机。

它只是把三件事拼起来：

  1. `dummy_vc_perception/vision_demo.launch.py`（URDF/TF、相机、vision_node）
  2. `scene_node`（把地面/地台发进 planning scene）
  3. `pick_place_node`（抓取规划 + 任务状态机）

**不含机械臂驱动、也不含 MoveIt** —— 按 docs/08 的做法，它们各开一个终端，
这样每层的日志和退出都各归各的。MoveIt 没起来时 `pick_place_node` 会报
"MoveIt 动作服务器没连上"（这是它该报的错，不是这个 launch 的问题）。

**相机/视觉已经单独起过时，务必加 `vision:=false`。** 否则这里会再起一套
`orbbec_camera` + `vision_node`：两个进程抢同一个 UVC 设备，第二个只会每十几秒
刷一轮 `Failed to initialize device uvc_open failed: [Path: 2-1-2.0, Return
Code: -6] status:115` / `Resetting device UID:`（`-6` = ENXIO，设备被占用），
还会多一个 `robot_state_publisher` 和 `moveit_hardware.launch.py` 发同名 TF。
2026-10-22 真机就是这么撞的，见 docs/04 的 G1。

`scene_node` 不能少：地面和地台不在 URDF 里，没有它们，规划器认为脚下什么
都没有，最后那段"往下抓"可以规划出一条从地面里穿过去的路径。

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
            'observe_timeout_s', default_value='60.0',
            description='等 /detected_objects 的上限（相机要十几秒才起来，别设太小）'),
        DeclareLaunchArgument(
            'place_x', default_value='0.0', description='放置点 x（base_link 下，米）'),
        DeclareLaunchArgument(
            'place_y', default_value='0.10', description='放置点 y（base_link 下，米）'),
        DeclareLaunchArgument(
            'scene', default_value='true',
            description='是否起 scene_node（把地面/地台发进 planning scene）'),
        DeclareLaunchArgument(
            'table_z', default_value='-0.0405',
            description='工作面的高度（base_link 下的 z，米）'),
    ]

    scene = Node(
        package='dummy_vc_grasp',
        executable='scene_node',
        name='scene_objects',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'table_z': ParameterValue(LaunchConfiguration('table_z'),
                                          value_type=float),
            },
        ],
        condition=IfCondition(LaunchConfiguration('scene')),
    )

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
                'observe_timeout_s': ParameterValue(
                    LaunchConfiguration('observe_timeout_s'), value_type=float),
            },
        ],
    )

    return LaunchDescription(args + [vision, scene, pick_place])
