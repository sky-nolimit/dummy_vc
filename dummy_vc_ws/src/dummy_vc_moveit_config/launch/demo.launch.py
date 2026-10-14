"""MoveIt demo: plan and drag in RViz WITHOUT the real arm.

Purpose
-------
Planning can be exercised with no hardware at all. `move_group` publishes the
planned trajectory and the fake controller consumes it, so the model follows the
plan in RViz. That separates "the planner is configured correctly" from "the
device accepts the trajectory" -- two failures that look identical from the
outside when you go straight to hardware.

If this launch shows the arm moving along planned paths in RViz but the hardware
launch does not, the fault is in the controller interface or the driver, not in
the MoveIt configuration.

    ros2 launch dummy_vc_moveit_config demo.launch.py

Unlike the hardware launch, this one runs `joint_state_publisher_gui`, because
there is no driver publishing /joint_states. Drag the sliders to set a start
state, then use the MotionPlanning panel.

    ros2 launch dummy_vc_moveit_config demo.launch.py gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    gui_arg = DeclareLaunchArgument('gui', default_value='true',
                                    description='run joint_state_publisher_gui')
    use_gui = LaunchConfiguration('gui')

    # Hand the robot description around explicitly. The driver package does the
    # same thing, and doing it here keeps this launch independent of whether a
    # driver is running.
    robot_description = {'robot_description':
                         moveit_config.robot_description['robot_description']}

    nodes = [
        gui_arg,

        # Publishes the fixed transforms and, fed by /joint_states, the moving
        # ones. This is what puts the arm in RViz.
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[robot_description]),

        # No driver here, so the start state has to come from somewhere.
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             condition=IfCondition(use_gui),
             output='screen'),
        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             condition=UnlessCondition(use_gui),
             output='screen'),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz_config],
             parameters=[
                 moveit_config.robot_description,
                 moveit_config.robot_description_semantic,
                 moveit_config.robot_description_kinematics,
                 moveit_config.planning_pipelines,
                 moveit_config.joint_limits,
             ],
             output='screen'),

        # The planner. With no controller manager configured for hardware, the
        # fake controller accepts whatever is planned.
        Node(package='moveit_ros_move_group',
             executable='move_group',
             output='screen',
             parameters=[moveit_config.to_dict()] +
                        [{'publish_robot_description_semantic': True}]),
    ]

    return LaunchDescription(nodes)
