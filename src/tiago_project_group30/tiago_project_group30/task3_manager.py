# Autonomous Mobile Robotics Exam - Group 30

# Task 3 - main manager node.

# Brings together all individual robot control modules
# (navigation , arm, localization, aruco tracking, costmap sampling, pick and place) 
# and the main state machine.

# New components for Task 3:
    # - GripperController (tiago_gripper.py)
    # - LinkAttacher (task3_link_attacher.py)
    # - CubeTracker (task3_cube_tracker.py)


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

        # ---------- 
        # callbacks
        # ----------
        cb_group_arm = ReentrantCallbackGroup()
        cb_group_nav = ReentrantCallbackGroup()
        cb_group_io = ReentrantCallbackGroup()
        cb_group_gripper = ReentrantCallbackGroup()

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
        self.gripper = GripperController(self, cb_group_gripper)
        self.link_attacher = LinkAttacher(self, cb_group_io)
        self.cube_tracker = CubeTracker(
            self, self.tf_buffer, self.amcl, cb_group_io
        )

        # ---------- 
        # State Machine
        # ----------
        self.state_machine = Task3StateMachine(
            self,
            self.arm, self.nav, self.amcl, self.aruco, self.sampler,
            self.gripper, self.link_attacher, self.cube_tracker,
            self.tf_buffer,
        )


def main(args=None):
    rclpy.init(args=args)
    node = Task3Manager()
    run_task(node, node.state_machine, num_threads=5) 


if __name__ == "__main__":
    main()
