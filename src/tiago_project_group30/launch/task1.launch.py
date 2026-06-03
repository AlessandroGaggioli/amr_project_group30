# Autonomous Mobile Robotics Exam - Group 30

# Task 1 launch file
#
# Launches:
#   - Gazebo simulation of the group30 world (with MoveIt)
#   - Nav2 navigation stack in SLAM mode (for map generation)
#   - task1_manager: tucks the arm to HOME, then exits
#   - explore_lite: started 5 s AFTER task1_manager exits, drives the
#     autonomous frontier exploration that builds the map
#
# Shared Gazebo / Nav2 includes  in tiago_project_group30.launch_common.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from tiago_project_group30.launch_common import tiago_sim, nav_bringup


def generate_launch_description():

    # Gazebo + MoveIt (group30 world) and Nav2 with SLAM (map generation).
    sim = tiago_sim()
    slam_nav = nav_bringup(slam=True)

    # explore_lite config (frontier exploration tuning).
    explore_config = os.path.join(
        get_package_share_directory('tiago_project_group30'),
        'config',
        'explore_lite.yaml',
    )

    # Arm -> HOME, then exits so explore can start with a clean profile.
    task1_manager = Node(
        package='tiago_project_group30',
        executable='task1_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_config,
            {'use_sim_time': True},
        ],
    )

    # Start explore_lite 5 s after the arm-tuck manager exits.
    explore = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=task1_manager,
            on_exit=[TimerAction(period=5.0, actions=[explore_node])],
        )
    )

    return LaunchDescription([
        sim,
        slam_nav,
        task1_manager,
        explore,
    ])
