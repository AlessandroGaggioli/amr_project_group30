# Autonomous Mobile Robotics Exam - Group 30

# Task 1 - manager node 
#
# Entrypoint: move the robot's arm to the home position. 
# Once the arm is safely tucked, the node ends and exits. 
# After it, the main launch file starts explore_lite package. 

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from tiago_project_group30.tiago_arm import ArmController
from tiago_project_group30.task1_state_machine import Task1StateMachine
from tiago_project_group30.task_runner import run_task


class Task1Manager(Node):

    def __init__(self):
        super().__init__("task1_manager")
        cb_group_arm = ReentrantCallbackGroup()
        self.arm = ArmController(self, cb_group_arm)
        self.state_machine = Task1StateMachine(self, self.arm)


def main(args=None):
    rclpy.init(args=args)
    node = Task1Manager()
    run_task(node, node.state_machine, num_threads=4)


if __name__ == "__main__":
    main()
