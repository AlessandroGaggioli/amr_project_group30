# TASK 3 of Autonomous Mobile Robotics Exam - Group 30
#
# Launches:
#   - Gazebo simulation of the group30 world (with MoveIt) -- inherits the
#     IFRA gazebo_ros_link_attacher world plugin loaded by the tiago_exam
#     world, which provides /ATTACHLINK and /DETACHLINK.
#   - Nav2 navigation stack with the map saved in Task 1.
#   - FOUR aruco_single detectors (vs. two in task2.launch.py):
#         * marker_id=26  (pick wall)  -> TF: aruco_pick_frame
#         * marker_id=238 (place wall) -> TF: aruco_place_frame
#         * marker_id=63  (top of pick cube #1) -> TF: aruco_cube_63_frame
#         * marker_id=582 (top of pick cube #2) -> TF: aruco_cube_582_frame
#   - task3_manager state-machine node, started after a delay so AMCL,
#     Nav2 and MoveIt are all up before the manager teleports the robot
#     and sends the first nav goal.

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
    # 3) ArUco detectors -- four instances, one per marker ID.
    #
    # All instances share the camera_info + image topics. They run in
    # separate namespaces so their per-node services / topics do not
    # collide; each is given a unique marker_frame so their TFs are
    # distinguishable.
    #
    # marker_size:
    #   - 0.25 m for the 25 cm pick / place WALL markers (exam spec).
    #   - 0.07 m for the 7 cm cube markers (exam spec).
    # ------------------------------------------------------------------
    aruco_common_remappings = [
        ('/camera_info', '/head_front_camera/rgb/camera_info'),
        ('/image',       '/head_front_camera/rgb/image_raw'),
    ]

    def _aruco_single_node(namespace, marker_id, marker_frame, marker_size):
        return Node(
            package='aruco_ros',
            executable='single',
            name='aruco_single',
            namespace=namespace,
            parameters=[{
                'image_is_rectified': True,
                'marker_size': marker_size,
                'marker_id': marker_id,
                'reference_frame': '',
                'camera_frame': 'head_front_camera_rgb_optical_frame',
                'marker_frame': marker_frame,
                'corner_refinement': 'LINES',
                'use_sim_time': True,
            }],
            remappings=aruco_common_remappings,
            output='screen',
        )

    aruco_pick = _aruco_single_node(
        namespace='aruco_pick',
        marker_id=26,
        marker_frame='aruco_pick_frame',
        marker_size=0.25,
    )
    aruco_place = _aruco_single_node(
        namespace='aruco_place',
        marker_id=238,
        marker_frame='aruco_place_frame',
        marker_size=0.25,
    )
    aruco_cube_63 = _aruco_single_node(
        namespace='aruco_cube_63',
        marker_id=63,
        marker_frame='aruco_cube_63_frame',
        marker_size=0.07,
    )
    aruco_cube_582 = _aruco_single_node(
        namespace='aruco_cube_582',
        marker_id=582,
        marker_frame='aruco_cube_582_frame',
        marker_size=0.07,
    )

    # ------------------------------------------------------------------
    # 4) Task 3 state machine
    # ------------------------------------------------------------------
    task3_manager = Node(
        package='tiago_project_group30',
        executable='task3_manager',
        name='task3_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    delayed_task3_manager = TimerAction(
        period=15.0,
        actions=[task3_manager],
    )

    return LaunchDescription([
        tiago_sim,
        slam_nav,
        aruco_pick,
        aruco_place,
        aruco_cube_63,
        aruco_cube_582,
        delayed_task3_manager,
    ])
