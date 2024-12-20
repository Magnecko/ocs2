
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ocs2_robotic_assets_dir = get_package_share_directory('ocs2_robotic_assets')
    ocs2_legged_robot_dir = get_package_share_directory('ocs2_legged_robot')
    ocs2_legged_robot_ros_dir = get_package_share_directory('ocs2_legged_robot_ros')

    task_file = os.path.join(ocs2_legged_robot_dir, 'config', 'mpc', 'task.info')
    reference_file = os.path.join(ocs2_legged_robot_dir, 'config', 'command', 'reference.info')
    urdf_file = os.path.join(ocs2_robotic_assets_dir, 'resources', 'magnecko', 'urdf', 'magnecko.urdf')
    gait_command_file = os.path.join(ocs2_legged_robot_dir, 'config', 'command', 'gait.info')
    rviz_config_file = os.path.join(ocs2_legged_robot_ros_dir, 'rviz', 'legged_robot.rviz')

    use_sim_time = False

    urdf_content = ""
    with open(urdf_file, 'r') as fp:
        urdf_content = fp.read()

    robot_description = {"robot_description": urdf_content}

    return LaunchDescription([

        Node(
            name='legged_robot_ipm_mpc',
            package='ocs2_legged_robot_ros',
            executable='legged_robot_ipm_mpc',
            parameters=[
                {"use_sim_time": use_sim_time},
                {"urdf_file": urdf_file},
                {"task_file": task_file},
                {"reference_file": reference_file},
                ],
            output='screen',
        ),
        Node(
            name='legged_robot_dummy',
            package='ocs2_legged_robot_ros',
            executable='legged_robot_dummy',
            parameters=[
                robot_description,
                {"use_sim_time": use_sim_time},
                {"urdf_file": urdf_file},
                {"task_file": task_file},
                {"reference_file": reference_file},
                ],
            output='screen',
            prefix=["gnome-terminal --"],
        ),
        Node(
            name='legged_robot_target',
            package='ocs2_legged_robot_ros',
            executable='legged_robot_target',
            parameters=[
                {"use_sim_time": use_sim_time},
                {"reference_file": reference_file},
                ],
            output='screen',
            prefix=["gnome-terminal --"],
        ),
        Node(
            name='legged_robot_gait_command',
            package='ocs2_legged_robot_ros',
            executable='legged_robot_gait_command',
            parameters=[
                {"use_sim_time": use_sim_time},
                {"gait_command_file": gait_command_file},
                ],
            output='screen',
            prefix=["gnome-terminal --"],
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [rviz_config_file]]
        )
    ])
