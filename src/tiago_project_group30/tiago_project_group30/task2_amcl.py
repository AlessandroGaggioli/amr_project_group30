# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# AMCL global-localization helper.
#
# Covers the standard ROS 2 / Nav2 workflow for "spawn the robot at a random
# pose and have it figure out where it is on the saved map":
#   1) Call /reinitialize_global_localization (std_srvs/Empty). AMCL drops
#      its single-modal estimate and re-spreads particles uniformly.
#   2) Drive a Nav2 /spin 2*pi so AMCL sees laser scans from every heading.
#   3) Monitor /amcl_pose covariance to detect convergence.
#
# Also owns the local/global costmap clearing services and the /spin action
# client (re-used by State 3 for the per-waypoint sweep).
#
# All futures are polled (no done_callbacks) from update_spin_flags() /
# update_amcl_flags(), mirroring Lab 4's update_arm_flags() pattern.

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

        # /initialpose publisher kept for compatibility with earlier code
        # paths that seeded AMCL from the spawn pose. With the global-
        # localization flow we no longer publish on it, but the publisher
        # is harmless and removing it would change the topic graph.
        self.initialpose_pub = node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # Latest AMCL pose (with covariance). Updated by _amcl_pose_cb every
        # time AMCL publishes. None until the first message arrives.
        self.pose = None
        # True once AMCL covariance has dropped below the convergence
        # thresholds (see update_amcl_flags). One-shot latch.
        self.converged = False

        # Sub-states for the AMCL localization phase (State 2 in the
        # state machine). See the State 2 block for the substate semantics.
        self.loc_substate = 0
        # How many Spin attempts we have launched for localization. We
        # cap at 3: each spin is ~10 s; after 3 unsuccessful attempts we
        # proceed anyway (AMCL is usually already close enough by then).
        self.localization_spin_count = 0

        # Bookkeeping for the Spin action. Re-used by State 3 for the
        # per-waypoint 2*pi sweep; the state machine inspects
        # spin_done/spin_succeeded and the in-flight goal handle.
        self.spin_active = False
        self.spin_done = False
        self.spin_succeeded = False
        self._spin_send_future = None
        self._spin_goal_handle = None
        self._spin_result_future = None

        self.global_loc_client = node.create_client(
            Empty, "/reinitialize_global_localization",
            callback_group=cb_group_nav,
        )
        self.spin_client = ActionClient(
            node, Spin, "/spin", callback_group=cb_group_nav,
        )
        # /local_costmap/clear_entirely_local_costmap is the standard Nav2
        # service used by the BT recovery node "ClearLocalCostmap". We
        # call it BEFORE each Spin retry so that obstacle markers
        # accumulated during the previous spin do not falsely trip the
        # "Collision Ahead" check (simulate_ahead_time) and abort the
        # new spin in 200 ms.
        self.clear_local_costmap_client = node.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=cb_group_nav,
        )
        # /global_costmap/clear_entirely_global_costmap wipes the "phantom
        # obstacle" marks the obstacle_layer accumulated while AMCL was
        # reporting the wrong pose (those marks would otherwise only clear
        # when the robot physically drives near them). The static_layer
        # (saved map) is a separate plugin and is NOT touched by this
        # service. We fire it once on the convergence transition.
        self.clear_global_costmap_client = node.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=cb_group_nav,
        )
        node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_cb,
            10,
            callback_group=cb_group_io,
        )

    # ------------------------------------------------------------------
    # Subscription callback
    # ------------------------------------------------------------------
    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        # Lab-style: store latest message; convergence is checked in
        # update_amcl_flags() during the state-machine tick.
        self.pose = msg

    # ------------------------------------------------------------------
    # Service requests
    # ------------------------------------------------------------------
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
        if not self.clear_local_costmap_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn(
                "/local_costmap/clear_entirely_local_costmap service unavailable"
            )
            return False
        self.clear_local_costmap_client.call_async(ClearEntireCostmap.Request())
        self.node.get_logger().info("Cleared local costmap (stale obstacle markers)")
        return True

    def clear_global_costmap(self) -> bool:
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

    # ------------------------------------------------------------------
    # Spin action (Nav2)
    # ------------------------------------------------------------------
    def send_spin(self, target_yaw: float = 2.0 * math.pi):
        # Standard Nav2 Spin action -- same one used internally by the
        # recovery BT. Used here both for localization (State 2) and
        # waypoint sweeps (State 3).
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
        if self._spin_goal_handle is not None:
            self._spin_goal_handle.cancel_goal_async()

    @property
    def spin_goal_handle(self):
        return self._spin_goal_handle

    # ------------------------------------------------------------------
    # Polling (called every state-machine tick)
    # ------------------------------------------------------------------
    def update_spin_flags(self):
        # Polling counterpart to NavClient.update_flags() but for /spin.
        if self._spin_send_future is not None and self._spin_send_future.done():
            try:
                handle = self._spin_send_future.result()
            except Exception:
                handle = None
            self._spin_send_future = None
            if handle is None or not handle.accepted:
                self.node.get_logger().warn("Nav2 rejected the Spin goal")
                self.spin_done = True
                self.spin_succeeded = False
                self.spin_active = False
            else:
                self._spin_goal_handle = handle
                self._spin_result_future = handle.get_result_async()

        if (
            self._spin_result_future is not None
            and self._spin_result_future.done()
        ):
            try:
                status = self._spin_result_future.result().status
            except Exception:
                status = 5  # treat as CANCELED
            self._spin_result_future = None
            self.spin_succeeded = (status == 4)
            self.spin_done = True
            self.spin_active = False
            self.node.get_logger().info(
                f"Spin finished, status={status}, "
                f"succeeded={self.spin_succeeded}"
            )

    def update_amcl_flags(self):
        # Convergence check based on /amcl_pose covariance. AMCL publishes
        # variance (m^2 / rad^2) along the diagonal of the 6x6 covariance.
        # Thresholds chosen so that:
        #   sqrt(0.10) ~= 0.32 m of position 1-sigma uncertainty
        #   sqrt(0.07) ~= 0.26 rad (~15 deg) of yaw 1-sigma uncertainty
        # AMCL after a successful spin typically reaches values an order
        # of magnitude smaller than this; we set the gate loosely so we
        # don't wait forever for the final tail to shrink.
        if self.pose is None:
            return
        cov = self.pose.pose.covariance
        cov_xx = cov[0]
        cov_yy = cov[7]
        cov_yaw = cov[35]
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
                # Clear global costmap on convergence latch so the random
                # search doesn't avoid phantom obstacles accumulated under
                # the wrong pre-convergence pose. Fired here (rather than
                # in State 2d) so it triggers once exactly at the
                # convergence event, independently of whether State 2d
                # had already advanced.
                self.clear_global_costmap()
