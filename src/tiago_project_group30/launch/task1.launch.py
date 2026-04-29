#TASK 1 of Autonomous Mobile Robotics Exam - Group 30 


from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
import os

def generate_launch_description():

    #Gazebo simulation with specific world (group 30 world)
    tiago_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tiago_exam'),'launch','tiago_exam.launch.py')]),
        launch_arguments={
            'world_name': 'group30',
            'moveit':'true'
        }.items()
    )
    
    #Navigation Stack with SLAM 

    #PER CAMBIARE I PARAMETRI DI NAV2
    # ~/tiago_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_params/params$ code tiago_nav2.yaml 
    #==========================================
    # slam nav original (tiago_2dnav -- tiago_nav_bringup.launch.py)
    #===========================================

    slam_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tiago_2dnav'),'launch','tiago_nav_bringup.launch.py')]),
            launch_arguments={
                'is_public_sim':'false',
                'rviz':'True',
                'slam':'True'
            }.items()
    )

    #Explore node (explore_lite)
    explore_config = os.path.join(
        get_package_share_directory('tiago_project_group30'),
        'config',
        'explore_lite.yaml'
    )

    arm_home_node = Node(
        package='tiago_project_group30',
        executable='task1_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_config,
            {'use_sim_time':True}
        ]
    )

    explore = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_home_node,
            on_exit=[explore_node],
        )
    )

    return LaunchDescription([
        tiago_sim,
        slam_nav,
        arm_home_node,
        explore
    ])
