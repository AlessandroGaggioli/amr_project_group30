# TASK 3 of Autonomous Mobile Robotics Exam - Group 30
#
# Thin wrapper around pymoveit2.GripperInterface, modeled on Lab 2's
# basic_gripper.py. Exposes a polling API (open() / close() + flags) in
# the same style as ArmController so the StateMachine can use it without
# blocking the executor thread.
#
# The exam spec requires the gripper open / close to be CLEARLY visible
# in simulation. The state machine therefore waits an extra short pause
# after each open/close call so the motion is unambiguously rendered in
# Gazebo before the next state runs (e.g. before the link attach fires).

from pymoveit2 import GripperInterface

from tiago_project_group30.task3_constants import (
    GRIPPER_CLOSED_POSITIONS,
    GRIPPER_COMMAND_ACTION_NAME,
    GRIPPER_GROUP_NAME,
    GRIPPER_JOINT_NAMES,
    GRIPPER_OPEN_POSITIONS,
)


class GripperController:

    def __init__(self, node, callback_group):
        self.node = node
        self.gripper = GripperInterface(
            node=node,
            gripper_joint_names=GRIPPER_JOINT_NAMES,
            open_gripper_joint_positions=GRIPPER_OPEN_POSITIONS,
            closed_gripper_joint_positions=GRIPPER_CLOSED_POSITIONS,
            gripper_group_name=GRIPPER_GROUP_NAME,
            callback_group=callback_group,
            gripper_command_action_name=GRIPPER_COMMAND_ACTION_NAME,
        )
        # One-shot latches the state machine polls.
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
        # GripperInterface exposes wait_until_executed() (blocking) but for
        # the non-blocking state machine we poll a public flag the
        # interface keeps internally. pymoveit2's GripperInterface inherits
        # from MoveIt2Gripper which sets the same EXECUTING/IDLE state as
        # MoveIt2, queried via query_state().
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
