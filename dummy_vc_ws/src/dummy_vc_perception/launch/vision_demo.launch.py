"""一条命令把"看"这一侧的三个节点起起来：URDF -> TF，相机，视觉节点。

手工等价的三条命令（出问题时用它们逐个排查，launch 会把日志混在一起）：

    xacro src/dummy2_urdf/urdf/dummy2_urdf.xacro > /tmp/dummy2.urdf
    ros2 run robot_state_publisher robot_state_publisher \
      --ros-args -p robot_description:="$(cat /tmp/dummy2.urdf)"
    ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true
    ros2 run dummy_vc_perception vision_node --ros-args \
      --params-file <share>/dummy_vc_perception/config/vision.yaml \
      -p image_topic:=/camera/color/image_raw \
      -p camera_info_topic:=/camera/color/camera_info

几个刻意的默认值：

* `use_depth:=false` —— 深度在这台相机上不可用（0.34 m 上只有 5% 像素有回波），
  单目是验证过的那条路。想再试就 `use_depth:=true`。
* `camera:=true` —— 相机由这个 launch 起。相机已经在别处跑着的时候传
  `camera:=false`，否则两个节点抢同一个设备。
* 只起"看"的那一半，**不起机械臂驱动**、也不起 MoveIt。这一层单独能验，
  这就是它存在的理由。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SHARE = get_package_share_directory('dummy_vc_perception')
DEFAULT_PARAMS = os.path.join(SHARE, 'config', 'vision.yaml')


def generate_launch_description():
    xacro_path = PathJoinSubstitution([
        FindPackageShare('dummy2_urdf_description'), 'urdf', 'dummy2_urdf.xacro'])

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=DEFAULT_PARAMS,
            description='视觉节点的参数文件'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='是否顺带起 Orbbec 相机（已经在别处跑着就传 false）'),
        DeclareLaunchArgument(
            'depth_registration', default_value='true',
            description='深度对齐到彩色。不对齐时深度图和彩图分辨率不同，像素会错位'),
        DeclareLaunchArgument(
            'use_depth', default_value='false',
            description='用深度算抓取点（这台相机上不可用，默认关）'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/color/image_raw',
            description='彩色图话题'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/color/camera_info',
            description='对应的 CameraInfo 话题'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='抓取点所在的坐标系'),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # xacro 里带 ${} 替换，必须整段当字符串交给 robot_description，
            # 不能让 launch 把它当 YAML 解析。
            'robot_description': ParameterValue(Command(['xacro ', xacro_path]),
                                                value_type=str),
        }],
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('orbbec_camera'),
            'launch', 'gemini_330_series.launch.py')),
        launch_arguments={
            'depth_registration': LaunchConfiguration('depth_registration'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    vision = Node(
        package='dummy_vc_perception',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'use_depth': ParameterValue(LaunchConfiguration('use_depth'),
                                            value_type=bool),
                'base_frame': LaunchConfiguration('base_frame'),
            },
        ],
    )

    return LaunchDescription(args + [robot_state_publisher, camera, vision])
