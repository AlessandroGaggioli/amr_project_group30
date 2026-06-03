#!/usr/bin/env python3
# Autonomous Mobile Robotics Exam - Group 30
#
# Task 2 - main manager node.
#
# Brings together all individual robot control modules 
# (navigation , arm, localization, aruco tracking, costmap sampling) and the main state machine.

# It runs the main StateMachine in a separate background thread. 
#
# Components: 
#   - ArmController   (tiago_arm.py)         
#   - NavClient       (task2_nav.py)       
#   - AmclLocalizer   (task2_amcl.py)      
#   - ArucoTracker    (task2_aruco.py)      
#   - CostmapSampler  (task2_costmap.py)  
#   - StateMachine    (task2_state_machine.py)
#
# Constants in constants.py
# Helpers in task2_kdl_helpers.py.

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tiago_project_group30.task_runner import run_task
from tiago_project_group30.task2_amcl import AmclLocalizer
from tiago_project_group30.tiago_arm import ArmController
from tiago_project_group30.task2_aruco import ArucoTracker
from tiago_project_group30.task2_costmap import CostmapSampler
from tiago_project_group30.task2_nav import NavClient
from tiago_project_group30.task2_state_machine import StateMachine


class Task2Manager(Node):

    def __init__(self):
        super().__init__("task2_manager")

        # ---------- 
        # callbacks
        # ----------
        cb_group_arm = ReentrantCallbackGroup()
        cb_group_nav = ReentrantCallbackGroup()
        cb_group_io = ReentrantCallbackGroup()

        # ---------- 
        # TF system 
        # ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------
        # Components 
        # ATTENTION: 
            # Order matters.
            #AMCL localizer must be created before ArucoTracker. 
        # ----------

        self.arm = ArmController(self, cb_group_arm)
        self.nav = NavClient(self, cb_group_nav)
        self.amcl = AmclLocalizer(self, cb_group_nav, cb_group_io)
        self.aruco = ArucoTracker(
            self, self.tf_buffer, self.tf_broadcaster, self.amcl, cb_group_io
        )
        self.sampler = CostmapSampler(self, self.tf_buffer, cb_group_io)

        # ---------- 
        # State Machine
        # ----------
        self.state_machine = StateMachine(
            self, self.arm, self.nav, self.amcl, self.aruco, self.sampler
        )


def main(args=None):
    rclpy.init(args=args)
    node = Task2Manager()
    run_task(node, node.state_machine, num_threads=4)


if __name__ == "__main__":
    main()
