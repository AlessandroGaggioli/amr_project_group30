# Autonomous Mobile Robotics Exam - Group 30
#
# Task 2 - AMCL Global Localization Helpers

# Workflow: 
    # Ask AMCL to spread particles everywhere on the map. 
    # Spin the robot in a full circle so its laser can scan the room. 
    # Monitor AMCL data unitil it's confident about location (convergence of covariance).

import math

from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import Spin
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import Empty


class AmclLocalizer:
    def __init__(self, node, cb_group_nav, cb_group_io):
        self.node = node

        # Not used. 
        self.initialpose_pub = node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        self.pose = None # Latest AMCL pose.
        self.converged = False # True if AMCL pose cov is converged. 
        self.loc_substate = 0 # Tracks substates of the localization process. 
        self.localization_spin_count = 0 # Numbers of spin tries. 

        # State tracking for spin action. 
        self.spin_active = False
        self.spin_done = False
        self.spin_succeeded = False
        self._spin_send_future = None
        self._spin_goal_handle = None
        self._spin_result_future = None

        # ----------
        # Service and Action Clients setup
        #------------

        # Service to tell AMCL to scatter its particles everywhere. 
        self.global_loc_client = node.create_client(
            Empty, "/reinitialize_global_localization",
            callback_group=cb_group_nav,
        )
        # Action to spin the robot. 
        self.spin_client = ActionClient(
            node, Spin, "/spin", callback_group=cb_group_nav,
        )

        # Service to clear short-term obstacle memory.
        # Ran before spinning to make sure the robot doesn't get stuck. 
        self.clear_local_costmap_client = node.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=cb_group_nav,
        )

        # Service to clear long-term obstacle memory. 
        # Ran once the robot figures out where it is, to erase ghost obstacles.
        self.clear_global_costmap_client = node.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=cb_group_nav,
        )

        # Subscribe to AMCL pose updates.
        node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_cb,
            10,
            callback_group=cb_group_io,
        )

    # ---------------------
    # Subscription callback
    # ---------------------
    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        # Save latest AMCL pose. 
        self.pose = msg

    # ---------------------
    # Service requests
    # ---------------------
    def request_global_localization(self) -> bool:
        # Service call: equivalent to running
        #   ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
        # AMCL replies after redistributing particles. 
        if not self.global_loc_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                "/reinitialize_global_localization service not available"
            )
            return False
        self.global_loc_client.call_async(Empty.Request())
        self.node.get_logger().info(
            "Requested AMCL /reinitialize_global_localization "
            "(uniform particle distribution)"
        )
        return True

    def clear_local_costmap(self) -> bool:
        # Clear the local (short-term) obstacle memory.
        if not self.clear_local_costmap_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn(
                "/local_costmap/clear_entirely_local_costmap service unavailable"
            )
            return False
        self.clear_local_costmap_client.call_async(ClearEntireCostmap.Request())
        self.node.get_logger().info("Cleared local costmap (stale obstacle markers)")
        return True

    def clear_global_costmap(self) -> bool:
        # Clear the global (long-term) obstacle memory. 
        if not self.clear_global_costmap_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn(
                "/global_costmap/clear_entirely_global_costmap service unavailable"
            )
            return False
        self.clear_global_costmap_client.call_async(ClearEntireCostmap.Request())
        self.node.get_logger().info(
            "Cleared global costmap (phantom obstacles from pre-convergence pose)"
        )
        return True

    # ---------------------
    # Spin action (Nav2)
    # ---------------------
    def send_spin(self, target_yaw: float = 2.0 * math.pi):
        # Standard Nav2 Spin action.
            # Spin in place. 
        self.spin_active = True
        self.spin_done = False
        self.spin_succeeded = False
        self._spin_send_future = None
        self._spin_goal_handle = None
        self._spin_result_future = None

        if not self.spin_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Nav2 /spin action server not available")
            self.spin_done = True
            self.spin_succeeded = False
            self.spin_active = False
            return

        goal_msg = Spin.Goal()
        goal_msg.target_yaw = target_yaw
        goal_msg.time_allowance = Duration(seconds=30.0).to_msg()
        self.node.get_logger().info(
            f"Sending Spin action goal: target_yaw={target_yaw:.2f} rad "
            f"({math.degrees(target_yaw):.1f} deg)"
        )
        self._spin_send_future = self.spin_client.send_goal_async(goal_msg)

    def cancel_spin(self):
        # Stops spin action if it's currently active.
        if self._spin_goal_handle is not None:
            self._spin_goal_handle.cancel_goal_async()

    # property to modify the method in only readable. 
    @property
    def spin_goal_handle(self):
        return self._spin_goal_handle

    # ---------------------
    # Polling (called every state-machine tick)
    # ---------------------
    def update_spin_flags(self):
        # Check if Spin action goal has started or finished.
            # Update the flags.

        # Checck if the nav server accepted the goal.
        if self._spin_send_future is not None and self._spin_send_future.done():
            try:
                handle = self._spin_send_future.result()
            except Exception:
                handle = None
            self._spin_send_future = None

            # Goal rejected. Mark the action as failed. 
            if handle is None or not handle.accepted:
                self.node.get_logger().warn("Nav2 rejected the Spin goal")
                self.spin_done = True
                self.spin_succeeded = False
                self.spin_active = False
            # Goal accepted.
            else:
                self._spin_goal_handle = handle
                self._spin_result_future = handle.get_result_async()

        # Check if the spin action finished. 
        if (
            self._spin_result_future is not None
            and self._spin_result_future.done()
        ):
            try:
                status = self._spin_result_future.result().status
            except Exception:
                status = 5  

            self._spin_result_future = None

            self.spin_succeeded = (status == 4) #status 4 means SUCCEEDED.
            self.spin_done = True 
            self.spin_active = False
            self.node.get_logger().info(
                f"Spin finished, status={status}, "
                f"succeeded={self.spin_succeeded}"
            )

    def update_amcl_flags(self):
        # Check AMCL covariance. 
            # If below threshold, consider AMCL converged and log the event.
        if self.pose is None:
            return
        cov = self.pose.pose.covariance
        cov_xx = cov[0]
        cov_yy = cov[7]
        cov_yaw = cov[35]

        #if cov is low enough.
        if cov_xx < 0.10 and cov_yy < 0.10 and cov_yaw < 0.07:
            if not self.converged:

                self.converged = True
                x = self.pose.pose.pose.position.x
                y = self.pose.pose.pose.position.y

                self.node.get_logger().info(
                    f"AMCL converged at map=({x:.2f}, {y:.2f}); "
                    f"cov_xx={cov_xx:.4f}, cov_yy={cov_yy:.4f}, "
                    f"cov_yaw={cov_yaw:.4f}"
                )
                # Clear global costmap now
                    #Once the robot is sure about its location.
                self.clear_global_costmap()
