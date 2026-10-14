"""MoveIt demo: plan and drag in RViz WITHOUT the real arm.

Purpose
-------
Planning can be exercised with no hardware at all. `move_group` accepts the
planned trajectory and the model follows the plan in RViz. That separates "the
planner is configured correctly" from "the device accepts the trajectory" --
two failures that look identical from the outside when you go straight to
hardware.

If this launch shows the arm moving along planned paths in RViz but the hardware
launch does not, the fault is in the controller interface or the driver, not in
the MoveIt configuration.

    ros2 launch dummy_vc_moveit_config demo.launch.py

Unlike the hardware launch, this one runs `joint_state_publisher_gui`, because
there is no driver publishing /joint_states. Drag the sliders to set a start
state, then use the MotionPlanning panel.

    ros2 launch dummy_vc_moveit_config demo.launch.py gui:=false

On this file's construction
---------------------------
It deliberately uses only the MoveItConfigsBuilder methods that are safe across
MoveIt releases, and it PRINTS what it loaded. An earlier version chained a
method whose parameter names differ between releases, and the resulting error
("Caught multiple exceptions when trying to load file") said nothing about the
cause. Anything version-sensitive is therefore either avoided or reported:

  * `planning_scene_monitor` is not configured through the builder; move_group
    uses its own defaults. Those defaults are adequate for planning, and the
    values that matter for hardware (allowed_start_tolerance, execution duration
    scaling) live in config/moveit_controllers.yaml, which IS loaded.
  * `moveit_config.to_dict()` is printed, so a missing or wrong entry is visible
    in the launch log instead of surfacing later as a mysterious planner failure.
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

    builder = MoveItConfigsBuilder(
        'dummy2',
        package_name='dummy_vc_moveit_config',
    ).robot_description_semantic(
        file_path='config/dummy2.srdf',
    ).joint_limits(
        file_path='config/joint_limits.yaml',
    ).trajectory_execution(
        file_path='config/moveit_controllers.yaml',
    )

    moveit_config = builder.to_moveit_configs()

    # Report what was actually loaded. Without this, a config file that failed
    # to load shows up much later as "planning failed", which points nowhere.
    robot_description = moveit_config.robot_description
    semantic = moveit_config.robot_description_semantic
    print('=' * 70)
    print('dummy_vc_moveit_config: what was loaded')
    print('=' * 70)
    urdf = robot_description.get('robot_description', '')
    print(f'  robot_description        : {len(urdf)} chars'
          f'{"  <-- EMPTY, check .setup_assistant" if len(urdf) < 100 else ""}')
    print(f'  robot_description_semantic: {len(str(semantic))} chars')
    print(f'  robot_description_kinematics groups: '
          f'{list(moveit_config.robot_description_kinematics.keys())}')
    print(f'  planning_pipelines       : '
          f'{list(moveit_config.planning_pipelines.keys())}')
    print(f'  trajectory_execution keys: '
          f'{sorted(moveit_config.trajectory_execution.keys())}')
    print('=' * 70)

    rviz_config = os.path.join(share, 'config', 'moveit.rviz')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='run joint_state_publisher_gui to set the start state')
    use_gui = LaunchConfiguration('gui')

    rsp_params = {'robot_description': urdf}

    nodes = [
        gui_arg,

        # Publishes the fixed transforms and, fed by /joint_states (or by
        # joint_state_publisher below), the moving ones.
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[rsp_params]),

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

        Node(package='moveit_ros_move_group',
             executable='move_group',
             output='screen',
             parameters=[moveit_config.to_dict()] +
                        [{'publish_robot_description_semantic': True}]),
    ]

    return LaunchDescription(nodes)
