# Autonomous Mobile Robotics Exam - Group 30
#
# Arm Controller module.

from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2 import MoveIt2, MoveIt2State

from tiago_project_group30.constants import (
    HOME_JOINT_POSITIONS,
    JOINT_ARM_NAMES,
)


class ArmController:
    def __init__(self, node, callback_group):
        self.node = node

        # Connection between the ArmController and MoveIt2. 
        self.arm = MoveIt2(
            node=node,
            joint_names=JOINT_ARM_NAMES,
            base_link_name="base_link",
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=callback_group,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"

        # Head controller (pan and tilt). 
        self.head_pub = node.create_publisher(
            JointTrajectory,
            "/head_controller/joint_trajectory",
            10,
        )

        self.motion_started = False
        self.motion_done = False

    def move_to_home(self, tilt_head: bool = False):
        #home positioning. 
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
            self.tilt_head(-0.5)

    def tilt_head(self, tilt_rad: float, pan_rad: float = 0.0,
                  time_from_start_s: float = 1.5):
        
        head_msg = JointTrajectory()
        head_msg.joint_names = ["head_1_joint", "head_2_joint"]

        point = JointTrajectoryPoint()
        point.positions = [pan_rad, tilt_rad]
        point.time_from_start = Duration(seconds=time_from_start_s).to_msg()
        head_msg.points.append(point)

        self.head_pub.publish(head_msg)

    def move_to_pose(self, position, quat_xyzw, cartesian=False):
        # Move the arm to a target pose.
        self.motion_started = False
        self.motion_done = False

        self.arm.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
            cartesian_max_step=0.01,
            cartesian_fraction_threshold=0.0,
        )

    def update_flags(self):
        # To check the status of the current arm motion. 
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
