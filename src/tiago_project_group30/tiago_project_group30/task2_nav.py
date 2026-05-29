# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# Nav2 NavigateToPose client, polled (no done_callbacks) so the state
# machine can decide per-tick whether to react to completion or to a
# timeout. Same Lab 4 pattern used by the arm controller and the spin
# polling inside the AMCL localizer.

import math

from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DriveOnHeading, NavigateToPose

from tiago_project_group30.task2_kdl_helpers import yaw_to_quat, quat_to_yaw


class NavClient:
    def __init__(self, node, callback_group, map_frame: str = "map"):
        self.node = node
        self.map_frame = map_frame
        self.client = ActionClient(
            node, NavigateToPose, "/navigate_to_pose", callback_group=callback_group
        )

        self.goal_active = False
        self.goal_done = False
        self.goal_succeeded = False
        self._goal_handle = None
        self._pending_send_future = None    # future from send_goal_async
        self._result_future = None          # future from handle.get_result_async
        # Set to True the moment the state machine emits a timeout warning
        # for the *current* goal, so it doesn't re-emit it every loop tick
        # while waiting for the cancel to propagate. Reset on every send.
        self.timeout_fired = False

        # /drive_on_heading is a Nav2 BEHAVIOR action that drives the base
        # in a straight line by a specified distance, IGNORING the global
        # costmap (no inflation check). Used in Task 3 to overcome the
        # 0.7 m inflation_radius that prevents Nav2's NavigateToPose from
        # parking close enough to the cube on the pick surface.
        self.drive_client = ActionClient(
            node, DriveOnHeading, "/drive_on_heading",
            callback_group=callback_group,
        )
        self.drive_active = False
        self.drive_done = False
        self.drive_succeeded = False
        self._drive_goal_handle = None
        self._drive_pending_send_future = None
        self._drive_result_future = None

    def send_goal(self, x: float, y: float, yaw: float):
        # Reset flags + drop any stale futures from a previous goal.
        # We don't register done_callbacks, so old futures dropped here
        # simply stop being polled and have no further side-effects on
        # this node (same logic as resetting MoveIt2 flags before
        # arm.move_to_pose() in Lab 4).
        self.goal_active = True
        self.goal_done = False
        self.goal_succeeded = False
        self.timeout_fired = False
        self._goal_handle = None
        self._pending_send_future = None
        self._result_future = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = yaw_to_quat(yaw)

        self.node.get_logger().info(
            f"Nav2 goal -> ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f} deg)"
        )

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Nav2 action server not available")
            self.goal_done = True
            self.goal_succeeded = False
            return

        # Store the future. NO add_done_callback -- we poll it.
        self._pending_send_future = self.client.send_goal_async(goal_msg)

    def update_flags(self):
        # Lab 4-style polling of the Nav2 action futures.
        #
        # Stage 1: did the SEND future complete (goal accepted/rejected)?
        if (
            self._pending_send_future is not None
            and self._pending_send_future.done()
        ):
            try:
                handle = self._pending_send_future.result()
            except Exception:
                handle = None
            self._pending_send_future = None
            if handle is None or not handle.accepted:
                self.node.get_logger().warn("Nav2 rejected the goal")
                self.goal_done = True
                self.goal_succeeded = False
                self.goal_active = False
            else:
                self._goal_handle = handle
                self._result_future = handle.get_result_async()

        # Stage 2: did the RESULT future complete (goal succeeded/failed)?
        if (
            self._result_future is not None
            and self._result_future.done()
        ):
            try:
                status = self._result_future.result().status
            except Exception:
                status = 5  # treat as CANCELED
            self._result_future = None
            # status == 4 -> SUCCEEDED in action_msgs/GoalStatus
            self.goal_succeeded = (status == 4)
            self.goal_done = True
            self.goal_active = False
            self.node.get_logger().info(
                f"Nav2 goal finished, status={status}, "
                f"succeeded={self.goal_succeeded}"
            )

    def cancel(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------
    # /drive_on_heading (Nav2 behavior action)
    # ------------------------------------------------------------------
    def send_drive_on_heading(self, distance: float,
                              speed: float = 0.1,
                              time_allowance_s: float = 30.0):
        # target.x is the forward offset in the base-link X direction.
        # speed in m/s. The action server publishes /cmd_vel internally
        # so we don't have to manage it ourselves.
        self.drive_active = True
        self.drive_done = False
        self.drive_succeeded = False
        self._drive_goal_handle = None
        self._drive_pending_send_future = None
        self._drive_result_future = None

        if not self.drive_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                "Nav2 /drive_on_heading action server not available"
            )
            self.drive_done = True
            self.drive_succeeded = False
            self.drive_active = False
            return

        goal_msg = DriveOnHeading.Goal()
        goal_msg.target.x = float(distance)
        goal_msg.target.y = 0.0
        goal_msg.target.z = 0.0
        goal_msg.speed = float(speed)
        goal_msg.time_allowance = Duration(seconds=time_allowance_s).to_msg()
        self.node.get_logger().info(
            f"/drive_on_heading -> {distance:.2f} m at {speed:.2f} m/s"
        )
        self._drive_pending_send_future = self.drive_client.send_goal_async(
            goal_msg
        )

    def update_drive_flags(self):
        # Same polling pattern used for /navigate_to_pose.
        if (
            self._drive_pending_send_future is not None
            and self._drive_pending_send_future.done()
        ):
            try:
                handle = self._drive_pending_send_future.result()
            except Exception:
                handle = None
            self._drive_pending_send_future = None
            if handle is None or not handle.accepted:
                self.node.get_logger().warn("Nav2 rejected the drive goal")
                self.drive_done = True
                self.drive_succeeded = False
                self.drive_active = False
            else:
                self._drive_goal_handle = handle
                self._drive_result_future = handle.get_result_async()

        if (
            self._drive_result_future is not None
            and self._drive_result_future.done()
        ):
            try:
                status = self._drive_result_future.result().status
            except Exception:
                status = 5
            self._drive_result_future = None
            self.drive_succeeded = (status == 4)
            self.drive_done = True
            self.drive_active = False
            self.node.get_logger().info(
                f"/drive_on_heading finished, status={status}, "
                f"succeeded={self.drive_succeeded}"
            )

    def cancel_drive(self):
        if self._drive_goal_handle is not None:
            self._drive_goal_handle.cancel_goal_async()

    @staticmethod
    def approach_pose_to_xy_yaw(ps: PoseStamped):
        x = ps.pose.position.x
        y = ps.pose.position.y
        yaw = quat_to_yaw(
            ps.pose.orientation.x,
            ps.pose.orientation.y,
            ps.pose.orientation.z,
            ps.pose.orientation.w,
        )
        return x, y, yaw
