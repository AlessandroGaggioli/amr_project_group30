# Autonomous Mobile Robotics Exam - Group 30

# Task 2 launch file
#
# Launches:
#   - Gazebo simulation of the group30 world (with MoveIt)
#   - Nav2 navigation stack with the map saved in Task 1
#   - Two aruco_single detectors:
#         - marker_id=26  (pick location)  -> TF: aruco_pick_frame
#         - marker_id=238 (place location) -> TF: aruco_place_frame
#   - task2_manager state-machine node, started after a delay so AMCL and
#     Nav2 are up before it sends /initialpose and the first nav goal.
#
# Shared Gazebo / Nav2 / aruco helpers in tiago_project_group30.launch_common.

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from tiago_project_group30.launch_common import (
    aruco_single_node,
    default_map_path,
    nav_bringup,
    tiago_sim,
)

def generate_launch_description():

    sim = tiago_sim()
    slam_nav = nav_bringup(map_path=default_map_path())

    # 25 cm wall markers 
    aruco_pick = aruco_single_node(
        "aruco_pick", 26, "aruco_pick_frame", 0.25)
    aruco_place = aruco_single_node(
        "aruco_place", 238, "aruco_place_frame", 0.25)

    # Started after a delay so AMCL / Nav2 / MoveIt are all up before the
    # manager teleports the robot and sends the first nav goal.
    task2_manager = Node(
        package='tiago_project_group30',
        executable='task2_manager',
        name='task2_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    delayed_task2_manager = TimerAction(period=15.0, actions=[task2_manager])

    return LaunchDescription([
        sim,
        slam_nav,
        aruco_pick,
        aruco_place,
        delayed_task2_manager,
    ])
