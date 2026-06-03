# Autonomous Mobile Robotics Exam - Group 30

# Task 3 launch file
#
# Launches:
#   - Gazebo simulation of the group30 world (with MoveIt) 
#     IFRA gazebo_ros_link_attacher world plugin loaded by the tiago_exam
#     world, which provides /ATTACHLINK and /DETACHLINK.
#   - Nav2 navigation stack with the map saved in Task 1.
#   - FOUR aruco_single detectors:
#         - marker_id=26  (pick wall)  -> TF: aruco_pick_frame
#         - marker_id=238 (place wall) -> TF: aruco_place_frame
#         - marker_id=63  (top of pick cube 1) -> TF: aruco_cube_63_frame
#         - marker_id=582 (top of pick cube 2) -> TF: aruco_cube_582_frame
#   - task3_manager state-machine node, started after a delay so AMCL,
#     Nav2 and MoveIt are all up before the manager teleports the robot
#     and sends the first nav goal.
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

    # 0.25 m wall markers + 0.07 m cube markers 
    aruco_pick = aruco_single_node(
        "aruco_pick", 26, "aruco_pick_frame", 0.25)
    aruco_place = aruco_single_node(
        "aruco_place", 238, "aruco_place_frame", 0.25)
    aruco_cube_63 = aruco_single_node(
        "aruco_cube_63", 63, "aruco_cube_63_frame", 0.07)
    aruco_cube_582 = aruco_single_node(
        "aruco_cube_582", 582, "aruco_cube_582_frame", 0.07)

    task3_manager = Node(
        package='tiago_project_group30',
        executable='task3_manager',
        name='task3_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    delayed_task3_manager = TimerAction(period=15.0, actions=[task3_manager])

    return LaunchDescription([
        sim,
        slam_nav,
        aruco_pick,
        aruco_place,
        aruco_cube_63,
        aruco_cube_582,
        delayed_task3_manager,
    ])
