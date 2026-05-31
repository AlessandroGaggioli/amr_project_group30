# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# Nav2 NavigateToPose client, polled (no done_callbacks) so the state
# machine can decide per-tick whether to react to completion or to a
# timeout. Same Lab 4 pattern used by the arm controller and the spin
# polling inside the AMCL localizer.

import math

from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose

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

        # Raw velocity publisher used in Task 3 to close the final gap to
        # the cube. Nav2's NavigateToPose obeys the 0.7 m inflation_radius
        # around the pick surface and parks ~1 m short of the cube, outside
        # the arm workspace. Instead of the fragile /drive_on_heading
        # behavior (open-loop, costmap-ignoring, easily aborted), the state
        # machine drives the base forward by publishing Twist here while
        # closed-loop checking the map->base_link TF, and calls stop() the
        # instant base_link reaches CUBE_APPROACH_DISTANCE from the cube.
        #
        # We publish to /nav_vel (NOT /cmd_vel). On Tiago the base velocity
        # chain is:
        #   Nav2 controller -> /cmd_vel -> velocity_smoother -> /nav_vel
        #   -> twist_mux -> /mobile_base_controller/cmd_vel_unstamped -> base
        # /cmd_vel only reaches the base while Nav2's velocity_smoother is
        # actively running a goal; a raw Twist published there when Nav2 is
        # idle gets swallowed by the smoother (the robot never moves). The
        # twist_mux navigation input /nav_vel is the topic actually
        # forwarded to the base controller, so we drive the base there.
        self.cmd_vel_pub = node.create_publisher(Twist, "/nav_vel", 10)

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
    # Raw /cmd_vel forward push (closed-loop, driven by the state machine)
    # ------------------------------------------------------------------
    def publish_forward(self, speed: float, angular: float = 0.0):
        # Drive along base_link +X at `speed` m/s with an optional yaw rate
        # `angular` rad/s (positive = turn left). The state machine calls
        # this every tick while it watches the TF pose relative to the
        # cube; one Twist per tick keeps the base moving. The angular term
        # lets the push steer toward the cube so the robot ends up frontal
        # and centered even if Nav2 parked it off-axis.
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        # Publish a zero Twist to halt the base immediately.
        self.cmd_vel_pub.publish(Twist())

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
