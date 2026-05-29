# Autonomous Mobile Robotics Exam - Group 30
#
# Shared ArmController for Task 1 and Task 2.
# Wraps MoveIt2 for the arm_torso group. Optionally publishes a
# JointTrajectory to /head_controller/joint_trajectory for the head tilt
# (Task 2 only; Task 1 passes tilt_head=False to move_to_home).
#
# Polling pattern (Lab 4 style): the state machine calls update_flags() each
# tick; motion_started / motion_done are one-shot latches.

from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2 import MoveIt2, MoveIt2State

from tiago_project_group30.task2_constants import (
    HOME_JOINT_POSITIONS,
    JOINT_ARM_NAMES,
)


class ArmController:
    def __init__(self, node, callback_group):
        self.node = node
        self.arm = MoveIt2(
            node=node,
            joint_names=JOINT_ARM_NAMES,
            base_link_name="base_link",
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=callback_group,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"

        # The head joints (head_1_joint pan, head_2_joint tilt) are driven
        # by the Tiago "head_controller" -- a JointTrajectoryController
        # separate from the MoveIt2 "arm_torso" group. We publish a
        # JointTrajectory directly to its action goal topic, since this is
        # a fire-and-forget "go to a fixed configuration" with no need
        # for collision-aware planning.
        self.head_pub = node.create_publisher(
            JointTrajectory,
            "/head_controller/joint_trajectory",
            10,
        )

        self.motion_started = False
        self.motion_done = False

    def move_to_home(self, tilt_head: bool = False):
        log_msg = "Sending arm to HOME joint configuration"
        if tilt_head:
            log_msg += " + head to mild tilt down"
        self.node.get_logger().info(log_msg)
        self.motion_started = False
        self.motion_done = False
        self.arm.move_to_configuration(
            joint_positions=HOME_JOINT_POSITIONS,
            joint_names=JOINT_ARM_NAMES,
        )
        if tilt_head:
            # Tilt = -0.5 rad (~17 deg below horizon) keeps ArUco markers
            # on cube faces / walls inside the camera FOV at 1-3 m without
            # losing the horizontal field where most markers sit.
            self.tilt_head(-0.5)

    def tilt_head(self, tilt_rad: float, pan_rad: float = 0.0,
                  time_from_start_s: float = 1.5):
        # Publish a JointTrajectory to /head_controller/joint_trajectory.
        # Used by Task 2 (mild tilt, -0.5 rad) AND by Task 3 (steep tilt,
        # -1.0 rad) to bring cubes on the pick surface into the FOV.
        head_msg = JointTrajectory()
        head_msg.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [pan_rad, tilt_rad]
        point.time_from_start = Duration(seconds=time_from_start_s).to_msg()
        head_msg.points.append(point)
        self.head_pub.publish(head_msg)

    def move_to_pose(self, position, quat_xyzw):
        # Cartesian goal for the end-effector (gripper_grasping_frame),
        # planned by MoveIt2. Same call shape as lab3/3_move_arm.py.
        self.motion_started = False
        self.motion_done = False
        self.arm.move_to_pose(position=position, quat_xyzw=quat_xyzw)

    def update_flags(self):
        # The `not self.motion_done` guard makes the "finished" log fire
        # exactly once per motion (otherwise this branch matches every loop
        # tick once IDLE is reached and spams at ~20 Hz).
        state = self.arm.query_state()
        if not self.motion_started and state == MoveIt2State.EXECUTING:
            self.motion_started = True
            self.node.get_logger().info("Arm motion started")
        if (
            self.motion_started
            and not self.motion_done
            and state == MoveIt2State.IDLE
        ):
            self.motion_done = True
            self.node.get_logger().info("Arm motion finished")
