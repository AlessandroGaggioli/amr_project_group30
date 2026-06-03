#!/usr/bin/env python3
# TASK 3 of Autonomous Mobile Robotics Exam - Group 30
#
# Entrypoint for Task 3: wires together the Task 2 components (arm, nav,
# AMCL, wall-marker aruco tracker, costmap sampler) PLUS the three new
# Task 3 components (gripper, link attacher, cube marker tracker) into a
# single rclpy.node.Node, and runs the Task3StateMachine on its own
# thread (same Lab 4 non-blocking pattern used in task2_manager.py).

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
from tiago_project_group30.task3_cube_tracker import CubeTracker
from tiago_project_group30.task3_link_attacher import LinkAttacher
from tiago_project_group30.task3_state_machine import Task3StateMachine
from tiago_project_group30.tiago_gripper import GripperController


class Task3Manager(Node):

    def __init__(self):
        super().__init__("task3_manager")

        # ---------- callback groups ----------
        cb_group_arm = ReentrantCallbackGroup()
        cb_group_nav = ReentrantCallbackGroup()
        cb_group_io = ReentrantCallbackGroup()
        cb_group_gripper = ReentrantCallbackGroup()

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- Task 2 components (re-used as-is) ----------
        self.arm = ArmController(self, cb_group_arm)
        self.nav = NavClient(self, cb_group_nav)
        self.amcl = AmclLocalizer(self, cb_group_nav, cb_group_io)
        self.aruco = ArucoTracker(
            self, self.tf_buffer, self.tf_broadcaster, self.amcl, cb_group_io
        )
        self.sampler = CostmapSampler(self, self.tf_buffer, cb_group_io)

        # ---------- Task 3 components ----------
        self.gripper = GripperController(self, cb_group_gripper)
        self.link_attacher = LinkAttacher(self, cb_group_io)
        self.cube_tracker = CubeTracker(
            self, self.tf_buffer, self.amcl, cb_group_io
        )

        # ---------- state machine ----------
        self.state_machine = Task3StateMachine(
            self,
            self.arm, self.nav, self.amcl, self.aruco, self.sampler,
            self.gripper, self.link_attacher, self.cube_tracker,
            self.tf_buffer,
        )


def main(args=None):
    rclpy.init(args=args)
    node = Task3Manager()
    # 5 threads (vs. 4 for Task 2): Task 3 adds the gripper callback group
    # plus AttachLink/DetachLink service futures, all of which need to
    # spin concurrently with the arm and nav action feedback.
    run_task(node, node.state_machine, num_threads=5)


if __name__ == "__main__":
    main()
