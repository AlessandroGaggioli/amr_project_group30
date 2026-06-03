# Autonomous Mobile Robotics Exam - Group 30
#
# Task 3 - Gripper controller module.


from pymoveit2 import GripperInterface

from tiago_project_group30.constants import (
    GRIPPER_CLOSED_POSITIONS,
    GRIPPER_COMMAND_ACTION_NAME,
    GRIPPER_GROUP_NAME,
    GRIPPER_JOINT_NAMES,
    GRIPPER_OPEN_POSITIONS,
)


class GripperController:

    def __init__(self, node, callback_group):
        self.node = node

        # Init the connection to robot's gripper using MoveIt2. 
        self.gripper = GripperInterface(
            node=node,
            gripper_joint_names=GRIPPER_JOINT_NAMES,
            open_gripper_joint_positions=GRIPPER_OPEN_POSITIONS,
            closed_gripper_joint_positions=GRIPPER_CLOSED_POSITIONS,
            gripper_group_name=GRIPPER_GROUP_NAME,
            callback_group=callback_group,
            gripper_command_action_name=GRIPPER_COMMAND_ACTION_NAME,
        )

        # Status flags
        self.action_started = False
        self.action_done = False

    def open(self):
        self.node.get_logger().info("Gripper: OPEN")
        self.action_started = False
        self.action_done = False
        self.gripper.open()

    def close(self):
        self.node.get_logger().info("Gripper: CLOSE")
        self.action_started = False
        self.action_done = False
        self.gripper.close()

    def update_flags(self):
        # Check the state of the gripper action. 
        from pymoveit2 import MoveIt2State

        state = self.gripper.query_state()

        if not self.action_started and state == MoveIt2State.EXECUTING:
            self.action_started = True
            self.node.get_logger().info("Gripper motion started")
            
        if (
            self.action_started
            and not self.action_done
            and state == MoveIt2State.IDLE
        ):
            self.action_done = True
            self.node.get_logger().info("Gripper motion finished")
