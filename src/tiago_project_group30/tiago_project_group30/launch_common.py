# Autonomous Mobile Robotics Exam - Group 30
#
# Shared launch files for task1/2/3.launch.py. 

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# All aruco_single instances share the front RGB camera topics.
ARUCO_REMAPPINGS = [
    ("/camera_info", "/head_front_camera/rgb/camera_info"),
    ("/image",       "/head_front_camera/rgb/image_raw"),
]

# ------------------
# Gazebo + MoveIt
# ------------------
def tiago_sim(world_name="group30", moveit="true"):

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("tiago_exam"),
            "launch", "tiago_exam.launch.py")]),
        launch_arguments={
            "world_name": world_name,
            "moveit": moveit,
        }.items(),
    )

# -----------------------
# Nav2 bringup - 
# SLAM mode (Task 1) when slam=True,
# otherwise localization in a saved map at map_path (Task 2/3)
# -----------------------
def nav_bringup(slam=False, map_path=None, rviz="true"):

    args = {"is_public_sim": "false", "rviz": rviz}
    if slam:
        args["slam"] = "true"
    if map_path is not None:
        args["map_path"] = map_path
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("tiago_2dnav"),
            "launch", "tiago_nav_bringup.launch.py")]),
        launch_arguments=args.items(),
    )

# ------------------
# ArUco nodes
# single detector.
# Each instance runs in its own namespace and publishes a unique marker_frame TF 
# so multiple detectors (pick/place and cubes) do not collide.
# ------------------
def aruco_single_node(namespace, marker_id, marker_frame, marker_size):

    return Node(
        package="aruco_ros",
        executable="single",
        name="aruco_single",
        namespace=namespace,
        parameters=[{
            "image_is_rectified": True,
            "marker_size": marker_size,
            "marker_id": marker_id,
            "reference_frame": "",
            "camera_frame": "head_front_camera_rgb_optical_frame",
            "marker_frame": marker_frame,
            "corner_refinement": "LINES",
            "use_sim_time": True,
        }],
        remappings=ARUCO_REMAPPINGS,
        output="screen",
    )

# ------------------
# Map path 
# ------------------
def default_map_path():

    return os.path.join(
        get_package_share_directory("tiago_project_group30"), "maps")
