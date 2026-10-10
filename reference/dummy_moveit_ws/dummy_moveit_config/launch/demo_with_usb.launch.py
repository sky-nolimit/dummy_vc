import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取 dummy_moveit_config 的 share 路径
    dummy_moveit_config_dir = get_package_share_directory('dummy_moveit_config')

    # 包含本包下的 demo.launch.py
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dummy_moveit_config_dir, 'launch', 'demo.launch.py')
        )
    )

    # 启动串口发送脚本 (ExecuteProcess)
    # 使用 launch 参数指定脚本路径，默认通过环境变量获取工作空间路径
    usb_sender_script_arg = DeclareLaunchArgument(
        'usb_sender_script',
        default_value=os.path.join(
            os.environ.get('DUMMY_MOVEIT_WS', os.path.expanduser('~/dummy_moveit_ws')),
            'test_usb_sender.py'
        ),
        description='test_usb_sender.py 的完整路径'
    )

    usb_sender_node = ExecuteProcess(
        cmd=['python3', LaunchConfiguration('usb_sender_script')],
        output='screen'
    )

    return LaunchDescription([
        usb_sender_script_arg,
        demo_launch,
        usb_sender_node,
    ])
