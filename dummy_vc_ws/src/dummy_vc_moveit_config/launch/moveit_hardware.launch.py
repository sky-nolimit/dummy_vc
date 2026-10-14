"""Drive the REAL arm from MoveIt.

Run the driver first, in another terminal:

    ros2 launch dummy_controller dummy_controller.launch.py

Then:

    ros2 launch dummy_vc_moveit_config moveit_hardware.launch.py

This launch deliberately does NOT start the driver. The driver is a lifecycle
node that talks to the device over fibre, and only one process may hold that
link; starting it from here would make the bringup depend on hardware being
present before anything else can be inspected. Keeping the two separate means a
MoveIt configuration problem can be debugged while the arm is idle.

What this starts:
  * a second robot_state_publisher, fed by the DRIVER's /joint_states
  * move_group, with the controller interface pointing at the topic the driver
    subscribes to

Deliberately NOT started: joint_state_publisher. The driver owns /joint_states,
and a second publisher on that topic is exactly the kind of silent conflict that
is hard to see -- the model would jump between two sources.

    ros2 launch dummy_vc_moveit_config moveit_hardware.launch.py rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory('dummy_vc_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder('dummy2', package_name='dummy_vc_moveit_config')
        .robot_description_semantic(file_path='config/dummy2.srdf')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            file_path='config/planning_scene_monitor_parameters.yaml')
        .to_moveit_configs()
    )

    rviz_config = os.path.join(share, 'config', 'moveit.rviz')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                     description='start RViz')
    use_rviz = LaunchConfiguration('rviz')

    robot_description = {'robot_description':
                         moveit_config.robot_description['robot_description']}

    nodes = [
        rviz_arg,

        # Needed here even though the driver may already run one: this makes the
        # launch self-contained when the driver is started with its own
        # publishing disabled, and a duplicate robot_state_publisher on the SAME
        # name would clash -- so it is named explicitly below.
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='moveit_robot_state_publisher',
             output='screen',
             parameters=[robot_description]),

        Node(package='moveit_ros_move_group',
             executable='move_group',
             output='screen',
             parameters=[moveit_config.to_dict()] +
                        [{'publish_robot_description_semantic': True}]),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz_config],
             condition=IfCondition(use_rviz),
             parameters=[
                 moveit_config.robot_description,
                 moveit_config.robot_description_semantic,
                 moveit_config.robot_description_kinematics,
                 moveit_config.planning_pipelines,
                 moveit_config.joint_limits,
             ],
             output='screen'),
    ]

    return LaunchDescription(nodes)
