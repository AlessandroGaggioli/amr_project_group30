# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# Launches:
#   - Gazebo simulation of the group30 world (with MoveIt)
#   - Nav2 navigation stack with the map saved in Task 1 (map2)
#   - Two aruco_single detectors:
#         * marker_id=26  (pick location)  -> TF: aruco_pick_frame
#         * marker_id=238 (place location) -> TF: aruco_place_frame
#   - task2_manager state-machine node, started after a delay so AMCL and
#     Nav2 are up before it sends /initialpose and the first nav goal.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------
    # 1) Gazebo simulation + MoveIt (group30 world)
    # ------------------------------------------------------------------
    tiago_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tiago_exam'),
            'launch', 'tiago_exam.launch.py')]),
        launch_arguments={
            'world_name': 'group30',
            'moveit': 'true',
        }.items()
    )

    # ------------------------------------------------------------------
    # 2) Nav2 with the map generated in Task 1
    #    NOTE: update `map_path` if you saved the Task 1 map elsewhere.
    # ------------------------------------------------------------------
    slam_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tiago_2dnav'),
            'launch', 'tiago_nav_bringup.launch.py')]),
        launch_arguments={
            'is_public_sim': 'false',
            'rviz': 'true',
            'map_path': '/home/alessandrogaggioli/tiago_ws/src/'
                        'tiago_project_group30/maps',
        }.items()
    )

    # ------------------------------------------------------------------
    # 3) ArUco detectors -- one per marker ID
    #
    # Each instance of aruco_single normally publishes on /aruco_single/...,
    # so we put the two instances in different ROS namespaces and we give
    # each one a unique marker_frame so their TFs do not collide.
    #
    # marker_size = 0.25 m as specified in the exam (pick/place arucos are
    # 25 cm wide). camera_frame matches Tiago's front RGB camera.
    # ------------------------------------------------------------------
    aruco_common_remappings = [
        ('/camera_info', '/head_front_camera/rgb/camera_info'),
        ('/image',       '/head_front_camera/rgb/image_raw'),
    ]

    aruco_pick = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        namespace='aruco_pick',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.25,
            'marker_id': 26,
            'reference_frame': '',
            'camera_frame': 'head_front_camera_rgb_optical_frame',
            'marker_frame': 'aruco_pick_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True,
        }],
        remappings=aruco_common_remappings,
        output='screen',
    )

    aruco_place = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        namespace='aruco_place',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.25,
            'marker_id': 238,
            'reference_frame': '',
            'camera_frame': 'head_front_camera_rgb_optical_frame',
            'marker_frame': 'aruco_place_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True,
        }],
        remappings=aruco_common_remappings,
        output='screen',
    )

    # ------------------------------------------------------------------
    # 4) Task 2 state machine
    #    Started after a delay so AMCL / Nav2 / MoveIt are all up before
    #    the manager teleports the robot and sends the first nav goal.
    # ------------------------------------------------------------------
    task2_manager = Node(
        package='tiago_project_group30',
        executable='task2_manager',
        name='task2_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    delayed_task2_manager = TimerAction(
        period=15.0,
        actions=[task2_manager],
    )

    return LaunchDescription([
        tiago_sim,
        slam_nav,
        aruco_pick,
        aruco_place,
        delayed_task2_manager,
    ])
