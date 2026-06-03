# Autonomous Mobile Robotics Exam - Group 30
#
# Task 2 - Navigation client.
#
# Sends nav goal to Nav2 system.

# Instead of using background callbacks, the client is polled. 
# The main StateMachine calls nav.update_flags() every loop tick to check if the goal is done.
# Gives the StateMachine control to cancel or retry if a timeout is exceeded, or if the goal fails.

import math

from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose

from tiago_project_group30.task2_kdl_helpers import yaw_to_quat, quat_to_yaw


class NavClient:
    def __init__(self, node, callback_group, map_frame: str = "map"):
        self.node = node
        self.map_frame = map_frame

        # Setup the connection to Nav2 action server
        self.client = ActionClient(
            node, NavigateToPose, "/navigate_to_pose", callback_group=callback_group
        )

        # Trackers for the current navigation goal
        self.goal_active = False
        self.goal_done = False
        self.goal_succeeded = False
        self._goal_handle = None
        self._pending_send_future = None    
        self._result_future = None          

        # Prevents the timeout warning. 
        self.timeout_fired = False

        # cause to the inflation radius that stops the robot far from the cubes
        # the arm cannot reach it. 
        # Bypass Nav2 and manually inch the robot forward the grab the cube. 
        # Publish to /nav_vel instaed of /cmd_vel because Nav2 blocks /cmd_vel when it's idle.
        self.cmd_vel_pub = node.create_publisher(Twist, "/nav_vel", 10)

    def send_goal(self, x: float, y: float, yaw: float):
        # Send a new Anv2 goal. 

        # Clean any old flags. 
        self.goal_active = True
        self.goal_done = False
        self.goal_succeeded = False
        self.timeout_fired = False
        self._goal_handle = None
        self._pending_send_future = None
        self._result_future = None

        # Build the message 
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = yaw_to_quat(yaw)

        self.node.get_logger().info(
            f"Nav2 goal -> ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f} deg)"
        )

        # Make sure Nav2 is active and listening. 
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Nav2 action server not available")
            self.goal_done = True
            self.goal_succeeded = False
            return

        # Send the command asynchronously 
        self._pending_send_future = self.client.send_goal_async(goal_msg)

    def update_flags(self):
        # called by StateMachine to check progress. 

        # Checks if we have sent a request to Nav2 AND if Nav2 has replied to it.
        if (
            self._pending_send_future is not None
            and self._pending_send_future.done()
        ):
            try:
                handle = self._pending_send_future.result()
            except Exception:
                handle = None
            self._pending_send_future = None

            # Checks if the communication failed (handle is None) OR if Nav2 explicitly refused the goal.
            if handle is None or not handle.accepted:
                self.node.get_logger().warn("Nav2 rejected the goal")
                self.goal_done = True
                self.goal_succeeded = False
                self.goal_active = False
            else:
                self._goal_handle = handle
                self._result_future = handle.get_result_async()

        # Checks if we are currently tracking a physical movement AND if the movement has finally finished.
        if (
            self._result_future is not None
            and self._result_future.done()
        ):
            try:
                status = self._result_future.result().status
            except Exception:
                status = 5  # CANCELED
            self._result_future = None
            self.goal_succeeded = (status == 4) # status == 4 means SUCCEEDED 
            self.goal_done = True
            self.goal_active = False
            self.node.get_logger().info(
                f"Nav2 goal finished, status={status}, "
                f"succeeded={self.goal_succeeded}"
            )

    def cancel(self):
        # Stop the robot is it's driving. 
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def reset_latches(self):
        # Clear the trackers. 
        # Useful to prevent reacting to old flags when we are not sending a new goal. 
        self.goal_active = False
        self.goal_done = False
        self.goal_succeeded = False
        self.timeout_fired = False

    # ------------------
    # nav_vel commands
    # ------------------
    def publish_forward(self, speed: float, angular: float = 0.0):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def approach_pose_to_xy_yaw(ps: PoseStamped):
        # Extracts flat 2D coordinates (x,y,yaw) from a 3D PoseStamped message. 
        x = ps.pose.position.x
        y = ps.pose.position.y
        yaw = quat_to_yaw(
            ps.pose.orientation.x,
            ps.pose.orientation.y,
            ps.pose.orientation.z,
            ps.pose.orientation.w,
        )
        return x, y, yaw
