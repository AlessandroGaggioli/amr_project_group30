#!/usr/bin/env python3
# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# Non-blocking state machine (modeled on Lab4 state_machine_not_blocking.py).
# Fulfills the Task 2 specification:
#   1) Seeds AMCL /initialpose at the spawn pose (random spawn is deferred).
#   2) Tucks the arm to a safe HOME configuration via MoveIt2.
#   3) Searches the environment by sampling random reachable waypoints from
#      the live global costmap, with a "visited" memory so the robot does
#      not re-visit the same area. At each sampled (x, y) it does a 4-step
#      in-place rotation (0, +90, +180, -90 deg) so the camera scans 360
#      around the spot.
#   4) ArUco pipeline follows the Lab 3 pattern (lab3_solution/lab3):
#        - Each aruco_single node runs with reference_frame=''
#          -> publishes the marker pose in the camera optical frame.
#        - This node subscribes to /aruco_pick/transform and
#          /aruco_place/transform and stores them as PyKDL.Frame.
#        - A 5 Hz timer composes:
#              frame_approach = (map <- camera TF) * marker_in_camera
#                                                 * Translate(0, 0, APPROACH)
#              frame_approach.M.DoRotY(pi/2)
#          and broadcasts the result as `aruco_pick_approach` /
#          `aruco_place_approach` via tf2_ros.TransformBroadcaster.
#        - The first time each broadcast succeeds the (x, y, yaw) is latched
#          into self.pick_approach_pose / self.place_approach_pose for Nav2.
#   5) When both approach poses are latched, the robot drives to PICK then
#      PLACE using Nav2 NavigateToPose.

import math
import random
import time
from collections import deque
from threading import Thread, Event

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import OccupancyGrid
#######################################################################
# GROUP 30 - Task 2 random spawn / global localization
# Added Spin action (used to rotate the robot while AMCL re-distributes
# its particles) and std_srvs/Empty (the type of the
# /reinitialize_global_localization service exposed by Nav2's AMCL node).
#######################################################################
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import Empty
#######################################################################
# END GROUP 30
#######################################################################

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster

from pymoveit2 import MoveIt2, MoveIt2State

from PyKDL import Frame, Rotation, Vector


##############################
# Tiago parameters
##############################

JOINT_ARM_NAMES = [
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
]

# Same HOME configuration used in task1_manager: torso lifted + arm tucked.
HOME_JOINT_POSITIONS = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]

##############################
# Search strategy
##############################

# Memory radius: when sampling a new waypoint we reject any (x, y) that is
# within this distance of a previously-tried waypoint. The 2-yaw spin at
# each waypoint already scans a ~3 m FOV around the spot, so 1.5 m keeps
# adjacent waypoints non-overlapping and pushes the search to spread
# faster across the room.
VISITED_RADIUS = 1.5  # meters

# Sampling is BIASED toward cells close to the robot's current position.
# - PREFERRED_RANGE: cells beyond this get exponentially less likely.
# - SOFT_MAX_RANGE: hard cap; cells beyond this are excluded entirely
#   unless no closer cell is available (then the cap is dropped).
# This produces "expanding ring" exploration: the robot first surveys
# nearby cells, only jumping farther once close cells are visited.
PREFERRED_RANGE = 2.0  # meters
SOFT_MAX_RANGE = 3.0   # meters

# Don't sample cells closer than this many meters to the saved-map edge.
# The global NavFn planner expands a potential field around the goal that
# reaches a few inflation radii outward; if that expansion crosses the map
# boundary it logs "worldToMap failed: mx,my: ..., size_x,size_y: ..." once
# *per iteration* and we get tens of thousands of errors per plan attempt.
# 0.6 m is comfortably more than the inflation radius (0.45 m) used in
# tiago_nav2.yaml, so the planner never reaches out-of-bounds pixels.
EDGE_MARGIN = 0.6  # meters

#######################################################################
# GROUP 30 - waypoint sweep is now a single Nav2 /spin 2*pi action
# (see State 3 phase machine in run_state_machine). The previous
# WAYPOINT_YAW_STEPS sequence of three NavigateToPose goals was removed
# because each NavigateToPose costs ~5-7 s of BT setup; one Spin
# covers the same 360 deg sweep in ~7-10 s total.
#######################################################################

##############################
# ArUco / approach parameters
##############################

PICK_MARKER_FRAME_RAW = "aruco_pick_frame"          # published by aruco_single ID 26
PLACE_MARKER_FRAME_RAW = "aruco_place_frame"        # published by aruco_single ID 238

# Derived approach TFs we broadcast (lab3-style naming).
PICK_APPROACH_FRAME = "aruco_pick_approach"
PLACE_APPROACH_FRAME = "aruco_place_approach"

# Camera optical frame (same as the camera_frame parameter of aruco_single).
CAMERA_FRAME = "head_front_camera_rgb_optical_frame"

# Distance the robot stops in front of a marker (along marker +Z).
APPROACH_DISTANCE = 0.55  # meters

#######################################################################
# GROUP 30 - max ArUco detection distance
# ArUco PnP accuracy degrades rapidly with distance. For a 25 cm marker
# in a 640x480 image:
#   ~4.0 m -> ~45 px wide, ~30 cm PnP error (used to be our value)
#   ~3.0 m -> ~60 px wide, ~15-20 cm PnP error
#   ~2.5 m -> ~75 px wide, ~10-15 cm PnP error
# With APPROACH_DISTANCE lowered to 0.40 m for task 3 pick-and-place,
# the marker_in_map position needs to be accurate to ~15 cm so the
# approach pose lands inside the inflation-free corridor in front of
# the cube. 3.0 m is the sweet spot: enough accuracy for the closer
# approach without forcing the robot to crawl right up to every wall
# during the random search.
#######################################################################
MAX_DETECTION_DISTANCE = 3.5  # meters

##############################
# Helpers
##############################


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    # 2D yaw from a quaternion. Same as tf_transformations.euler_from_quaternion[2].
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_to_kdl_frame(tf_msg: TransformStamped) -> Frame:
    p = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    return Frame(Rotation.Quaternion(r.x, r.y, r.z, r.w), Vector(p.x, p.y, p.z))


##############################


class Task2Manager(Node):

    def __init__(self):
        super().__init__("task2_manager")

        self.robot_base_frame = "base_link"
        self.map_frame = "map"
        self.camera_frame = CAMERA_FRAME

        # ---------- state machine state ----------
        self.state = 0
        self.finished = False
        # While True, approach poses are continuously *replaced* with the
        # latest aruco observation (so noisy long-range first detections
        # are overwritten as the robot gets closer). Set to False once we
        # transition to State 4 so the nav goal does not chase a moving
        # estimate during the final approach.
        self._refresh_approach_poses = True

        # Arm motion bookkeeping (Lab4 non-blocking pattern)
        self.arm_motion_started = False
        self.arm_motion_done = False

        # Navigation bookkeeping. Polling pattern -- mirrors Lab 4's
        # update_arm_flags() which polls arm.query_state() each loop tick
        # instead of registering done_callbacks. We hold the in-flight
        # futures and let update_nav_flags() check their .done() each
        # state-machine iteration. No callbacks => no stale-callback race.
        self.nav_goal_active = False
        self.nav_goal_done = False
        self.nav_goal_succeeded = False
        self._nav_goal_handle = None
        self._pending_send_future = None    # future from nav_client.send_goal_async
        self._nav_result_future = None      # future from handle.get_result_async
        # Set to True the moment we emit a timeout warning for the *current*
        # goal, so we don't re-emit it every loop tick while we wait for the
        # cancel to propagate. Reset on every send_nav_goal.
        self._timeout_fired = False

        #######################################################################
        # GROUP 30 - Task 2 random spawn / global localization bookkeeping
        # Pattern: lab-style (subscriber for /amcl_pose, action client for
        # /spin, service client for /reinitialize_global_localization). All
        # futures are polled non-blocking like update_arm_flags() in Lab 4.
        #######################################################################
        # Latest AMCL pose (with covariance). Updated by _amcl_pose_cb every
        # time AMCL publishes. None until the first message arrives.
        self.amcl_pose = None
        # True once AMCL covariance has dropped below the convergence
        # thresholds (see update_amcl_flags). One-shot latch.
        self.amcl_converged = False

        # Sub-states for the AMCL localization phase (State 2 in the
        # state machine). See the State 2 block for the substate semantics.
        self.loc_substate = 0
        # How many Spin attempts we have launched for localization. We
        # cap at 3: each spin is ~10 s; after 3 unsuccessful attempts we
        # proceed anyway (AMCL is usually already close enough by then).
        self.localization_spin_count = 0
        # Bookkeeping for the Spin action (kept separate from nav_goal_*
        # so a search-nav-goal timeout cannot disturb the localization spin).
        self.spin_active = False
        self.spin_done = False
        self.spin_succeeded = False
        self._spin_send_future = None
        self._spin_goal_handle = None
        self._spin_result_future = None
        #######################################################################
        # END GROUP 30
        #######################################################################

        # Marker poses in MAP frame, composed at observation time.
        # We do NOT store the raw camera-frame pose and recompose later,
        # because doing so multiplies a *current* map<-camera TF by an
        # *old* marker_in_camera (taken when the robot was elsewhere) and
        # produces a frame that slides around the map as the robot moves.
        # Composing inside the subscription callback (with TF looked up at
        # msg.header.stamp) freezes the marker at its true world position.
        self.pick_marker_in_map = None     # PyKDL.Frame or None
        self.place_marker_in_map = None

        # Distance from the camera to the marker at the moment of the
        # observation that produced *_marker_in_map. ArUco pose accuracy
        # degrades fast with distance (a 25 cm marker at 5 m is only ~30
        # px wide, so its estimated depth/orientation can be off by a
        # meter+). We use these to KEEP THE CLOSEST OBSERVATION:
        # in _handle_aruco_detection we only overwrite *_marker_in_map
        # when the new observation is closer than the stored one. This
        # extends the Lab 3 pattern (where the robot is stationary and a
        # plain "latest observation wins" policy was enough) to the
        # moving-robot case in Task 2 -- without a filter, a single bad
        # late observation from a worse angle/distance could replace a
        # good earlier one.
        self.pick_detection_distance = float("inf")
        self.place_detection_distance = float("inf")

        # Latched approach poses in map frame (PoseStamped)
        self.pick_approach_pose = None
        self.place_approach_pose = None

        # Diagnostics: True once the aruco_single node has ever published a
        # transform for that marker (regardless of TF lookup success).
        self.pick_seen_in_camera = False
        self.place_seen_in_camera = False

        # ---------- callback groups ----------
        cb_group_arm = ReentrantCallbackGroup()
        cb_group_nav = ReentrantCallbackGroup()
        cb_group_io = ReentrantCallbackGroup()

        # ---------- MoveIt2 (arm) ----------
        self.arm = MoveIt2(
            node=self,
            joint_names=JOINT_ARM_NAMES,
            base_link_name=self.robot_base_frame,
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=cb_group_arm,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"

        # ---------- TF (lab3 uses a Buffer + TransformListener) ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Lab3 also uses a TransformBroadcaster to publish the derived
        # approach frames; we mirror that here.
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- AMCL /initialpose publisher ----------
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # ---------- Head trajectory publisher ----------
        # The head joints (head_1_joint pan, head_2_joint tilt) are driven
        # by the Tiago "head_controller" -- a JointTrajectoryController
        # separate from the MoveIt2 "arm_torso" group. We publish a
        # JointTrajectory directly to its action goal topic, since this is
        # a fire-and-forget "go to a fixed configuration" with no need
        # for collision-aware planning.
        self.head_pub = self.create_publisher(
            JointTrajectory,
            "/head_controller/joint_trajectory",
            10,
        )

        # ---------- Nav2 action client ----------
        self.nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=cb_group_nav
        )

        #######################################################################
        # GROUP 30 - global localization clients (Task 2 random spawn)
        # /reinitialize_global_localization is the standard Nav2 AMCL service
        # used to redistribute particles uniformly across the map. /spin is
        # the standard Nav2 behavior action; we run a full 2*pi rotation so
        # AMCL observes the environment from every direction.
        # /amcl_pose carries AMCL's current pose estimate with covariance,
        # which we monitor to detect convergence.
        #######################################################################
        self.global_loc_client = self.create_client(
            Empty, "/reinitialize_global_localization",
            callback_group=cb_group_nav,
        )
        self.spin_client = ActionClient(
            self, Spin, "/spin", callback_group=cb_group_nav,
        )
        # /local_costmap/clear_entirely_local_costmap is the standard Nav2
        # service used by the BT recovery node "ClearLocalCostmap". We
        # call it BEFORE each Spin retry so that obstacle markers
        # accumulated during the previous spin do not falsely trip the
        # "Collision Ahead" check (simulate_ahead_time) and abort the
        # new spin in 200 ms.
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=cb_group_nav,
        )
        # /global_costmap/clear_entirely_global_costmap is the equivalent
        # service for the global costmap. We call it ONCE right after AMCL
        # converges, to wipe the "phantom obstacle" marks the
        # obstacle_layer accumulated while AMCL was reporting the wrong
        # pose (those marks would otherwise only clear when the robot
        # physically drives near them, since clearing relies on raytraced
        # laser rays which have a finite raytrace_max_range). The
        # static_layer (saved map) is a separate plugin and is NOT touched
        # by this service.
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=cb_group_nav,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_cb,
            10,
            callback_group=cb_group_io,
        )
        #######################################################################
        # END GROUP 30
        #######################################################################

        # ---------- Global costmap subscription (for random sampling) ----------
        # The costmap is published with TRANSIENT_LOCAL durability so we use
        # the same QoS to make sure we get the latched message.
        self.costmap_msg = None
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._costmap_cb,
            costmap_qos,
            callback_group=cb_group_io,
        )

        # ---------- ArUco subscriptions (lab3 pattern) ----------
        # aruco_single runs with reference_frame='' -> publishes marker pose
        # in the CAMERA optical frame. The topic is private to the node so
        # with namespace='aruco_pick'/name='aruco_single' it resolves to:
        #     /aruco_pick/aruco_single/transform
        # NOT /aruco_pick/transform (that one only appears in `ros2 topic
        # list` because subscribing creates an entry, even with no publisher).
        self.create_subscription(
            TransformStamped, "/aruco_pick/aruco_single/transform",
            self._aruco_pick_cb, 10, callback_group=cb_group_io,
        )
        self.create_subscription(
            TransformStamped, "/aruco_place/aruco_single/transform",
            self._aruco_place_cb, 10, callback_group=cb_group_io,
        )

        # ---------- Approach-frame publisher timer (lab3 pattern) ----------
        # At 5 Hz, if we have a stored marker_in_camera frame, compose it
        # with map<-camera TF and broadcast aruco_pick_approach /
        # aruco_place_approach. The first successful composition latches
        # the corresponding approach pose for Nav2 navigation.
        self.approach_timer = self.create_timer(
            0.2, self.publish_approach_frames, callback_group=cb_group_io
        )

        # ---------- Random-sampling search state ----------
        # visited_waypoints stores (x, y) of every waypoint we have already
        # *attempted* (whether reached or timed out). Acts as memory so the
        # sampler doesn't repeatedly pick nearby cells.
        self.visited_waypoints = []
        #######################################################################
        # GROUP 30 - per-waypoint phase machine for the random search
        # Old design sent 3 NavigateToPose per waypoint (yaw 0, +120, -120),
        # ~25-30 s each: PLACE search took 9+ minutes. New design:
        #   1) drive once to (x, y, yaw=0)
        #   2) issue a Nav2 /spin 2*pi for the in-place sweep (~7-10 s)
        # If the Spin aborts (Collision Ahead in a tight pocket), we just
        # move on to the next waypoint -- the random search will keep
        # sweeping the map, and any marker missed at this waypoint will
        # likely be picked up at the next one.
        # Phases:
        #   'sample' -> idle, ready to draw a new (x, y)
        #   'nav'    -> NavigateToPose to the new waypoint in flight
        #   'spin'   -> /spin 2*pi in flight after arrival
        #######################################################################
        self.search_phase = 'sample'
        #######################################################################
        # END GROUP 30
        #######################################################################

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _costmap_cb(self, msg: OccupancyGrid):
        self.costmap_msg = msg

    def _aruco_pick_cb(self, msg: TransformStamped):
        self._handle_aruco_detection(
            msg, label="PICK", marker_id=26, is_pick=True
        )

    def _aruco_place_cb(self, msg: TransformStamped):
        self._handle_aruco_detection(
            msg, label="PLACE", marker_id=238, is_pick=False
        )

    def _handle_aruco_detection(
        self, msg: TransformStamped, label: str, marker_id: int, is_pick: bool
    ):
        #######################################################################
        # GROUP 30 - gate ArUco -> map composition on AMCL convergence
        # Until AMCL has converged, the TF map<-camera is built on top of
        # AMCL's prior (uniform / uninitialized), so composing the marker
        # pose with it gives a garbage map-frame position (we observed a
        # PICK approach landing inside a wall when an ArUco detection was
        # processed 1.3 s after /reinitialize_global_localization). Once
        # AMCL is locked we accept detections normally.
        #######################################################################
        if not self.amcl_converged:
            return
        #######################################################################
        # END GROUP 30
        #######################################################################
        # Convert marker_in_camera frame (raw aruco_single output).
        aruco_in_cam = Frame(
            Rotation.Quaternion(
                msg.transform.rotation.x, msg.transform.rotation.y,
                msg.transform.rotation.z, msg.transform.rotation.w,
            ),
            Vector(
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z,
            ),
        )

        # Diagnostic: log the first time we ever hear from this detector.
        seen_flag = "pick_seen_in_camera" if is_pick else "place_seen_in_camera"
        if not getattr(self, seen_flag):
            setattr(self, seen_flag, True)
            self.get_logger().info(
                f"[diag] aruco_single {label} (ID {marker_id}) IS seeing the marker"
            )

        # We only refresh the stored marker_in_map while still searching.
        # Once State 4 has frozen the approach poses we keep the latched
        # value so the nav goal target doesn't drift on us.
        already_have = (
            self.pick_marker_in_map if is_pick else self.place_marker_in_map
        ) is not None
        if already_have and not self._refresh_approach_poses:
            return

        # ArUco pose accuracy degrades with distance (smaller pixel area
        # of the marker -> noisier corner detection -> noisier PnP). The
        # camera-frame translation of the marker IS the distance from
        # camera to marker, so we compute it directly without any TF.
        new_distance = math.sqrt(
            aruco_in_cam.p.x() ** 2
            + aruco_in_cam.p.y() ** 2
            + aruco_in_cam.p.z() ** 2
        )
        #######################################################################
        # GROUP 30 - reject detections beyond MAX_DETECTION_DISTANCE
        # (see header comment on MAX_DETECTION_DISTANCE for rationale).
        #######################################################################
        if new_distance > MAX_DETECTION_DISTANCE:
            return
        #######################################################################
        # END GROUP 30
        #######################################################################
        # KEEP-CLOSEST policy: only overwrite the stored marker_in_map if
        # this new observation is closer than the previous one.
        prev_distance = (
            self.pick_detection_distance if is_pick
            else self.place_detection_distance
        )
        if already_have and new_distance >= prev_distance:
            return  # this observation is no closer than what we already have

        # CRITICAL: compose into MAP frame NOW, using the camera TF
        # AT THE TIME OF THE DETECTION (msg.header.stamp). This freezes
        # the marker at its true world position. Recomposing later with a
        # fresh map<-camera TF would make the result slide as the camera
        # moves -- that was the "approach moves infinitely" bug.
        try:
            tf_map_cam = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            # Fallback: TF for that exact stamp not available -> latest.
            try:
                tf_map_cam = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.camera_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
            except Exception:
                return  # no TF available, skip this detection

        frame_map_cam = transform_to_kdl_frame(tf_map_cam)
        marker_in_map = frame_map_cam * aruco_in_cam

        if is_pick:
            self.pick_marker_in_map = marker_in_map
            self.pick_detection_distance = new_distance
        else:
            self.place_marker_in_map = marker_in_map
            self.place_detection_distance = new_distance

    # ------------------------------------------------------------------
    # Approach-frame broadcaster (lab3-style, adapted for 2-D navigation)
    # ------------------------------------------------------------------
    # Lab 3 computes the approach by offsetting APPROACH_DISTANCE along the
    # marker's Z axis and then applying DoRotY(pi/2) so the frame's X axis
    # points toward the marker.  That works well for the arm approach (Task 3)
    # but for 2-D base navigation the resulting frame has non-zero pitch/roll
    # (because the marker's Y is vertical/down), which looks wrong in RViz
    # and produces an awkward Nav2 goal orientation.
    #
    # Instead we project the marker's Z axis onto the horizontal XY plane to
    # get the approach direction, then build the approach frame with a flat
    # orientation (roll=0, pitch=0, yaw toward marker).  The position keeps
    # the marker's Z height so Task 3 can reuse the same frame.
    #
    # The marker pose in MAP frame is already stored stably by the
    # subscription callback (see _handle_aruco_detection). Here we just
    # compute the approach offset and broadcast the TF -- no per-tick
    # TF lookup, no recomposition.
    def publish_approach_frames(self):
        if self.pick_marker_in_map is not None:
            approach = self._compute_horizontal_approach(
                self.pick_marker_in_map, APPROACH_DISTANCE
            )
            self._broadcast_frame(approach, PICK_APPROACH_FRAME)
            first_time = self.pick_approach_pose is None
            if first_time or self._refresh_approach_poses:
                self.pick_approach_pose = self._frame_to_pose_stamped(approach)
                if first_time:
                    self.get_logger().info(
                        f"PICK approach localized at map "
                        f"({approach.p.x():.2f}, {approach.p.y():.2f})"
                    )

        if self.place_marker_in_map is not None:
            approach = self._compute_horizontal_approach(
                self.place_marker_in_map, APPROACH_DISTANCE
            )
            self._broadcast_frame(approach, PLACE_APPROACH_FRAME)
            first_time = self.place_approach_pose is None
            if first_time or self._refresh_approach_poses:
                self.place_approach_pose = self._frame_to_pose_stamped(approach)
                if first_time:
                    self.get_logger().info(
                        f"PLACE approach localized at map "
                        f"({approach.p.x():.2f}, {approach.p.y():.2f})"
                    )

    def _compute_horizontal_approach(self, marker_in_map: Frame, distance: float) -> Frame:
        # The marker's Z axis in map frame: points toward the camera/robot
        # (out of the marker face -- standard aruco_ros convention).
        marker_z = marker_in_map.M * Vector(0.0, 0.0, 1.0)

        # Project to the horizontal plane (ignore Z component).
        horiz = math.sqrt(marker_z.x() ** 2 + marker_z.y() ** 2)
        if horiz > 0.05:
            dx = marker_z.x() / horiz
            dy = marker_z.y() / horiz
        else:
            # Fallback for nearly-vertical Z (top-mounted marker): use +X.
            dx, dy = 1.0, 0.0

        # Approach position: offset from marker in the marker-toward-robot direction.
        ax = marker_in_map.p.x() + distance * dx
        ay = marker_in_map.p.y() + distance * dy
        az = marker_in_map.p.z()  # keep marker height (useful for Task 3 arm approach)

        # Yaw so the robot faces the marker when it arrives at the approach pose.
        yaw = math.atan2(marker_in_map.p.y() - ay, marker_in_map.p.x() - ax)

        # Flat orientation (roll=0, pitch=0) so Z points up in the map frame:
        # visually clean in RViz and correct for Nav2's 2-D heading.
        return Frame(Rotation.RPY(0.0, 0.0, yaw), Vector(ax, ay, az))

    def _broadcast_frame(self, frame: Frame, child_frame_id: str):
        # Same pattern as lab3's publish_frame().
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = child_frame_id
        t.transform.translation.x = frame.p.x()
        t.transform.translation.y = frame.p.y()
        t.transform.translation.z = frame.p.z()
        qx, qy, qz, qw = frame.M.GetQuaternion()
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        t.transform.rotation.x = qx / n
        t.transform.rotation.y = qy / n
        t.transform.rotation.z = qz / n
        t.transform.rotation.w = qw / n
        self.tf_broadcaster.sendTransform(t)

    def _frame_to_pose_stamped(self, frame: Frame) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = frame.p.x()
        ps.pose.position.y = frame.p.y()
        ps.pose.position.z = frame.p.z()
        qx, qy, qz, qw = frame.M.GetQuaternion()
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        ps.pose.orientation.x = qx / n
        ps.pose.orientation.y = qy / n
        ps.pose.orientation.z = qz / n
        ps.pose.orientation.w = qw / n
        return ps

    # ------------------------------------------------------------------
    # Random-waypoint sampling from the global costmap
    # ------------------------------------------------------------------
    # The static map can contain "free islands": clusters of cells whose
    # OccupancyGrid value is 0 (free), but which are disconnected from the
    # room the robot actually navigates in. Reasons range from SLAM artifacts
    # (laser briefly glimpsing past a wall, ghost echoes) to the map saver
    # writing the full bounding rectangle.
    #
    # If we sample one of those islands the global planner will correctly
    # refuse to make a plan and the robot waits a NAV_TIMEOUT for nothing
    # (and worse, the recovery behaviors -- backup, spin -- can drift the
    # robot into off-camera obstacles like the kitchen counter).
    #
    # The fix is a flood-fill from the robot's current cell: only the cells
    # in the same 4-connected free component as the robot are eligible.
    # ------------------------------------------------------------------
    def _compute_reachable_mask(self):
        # Returns a (H, W) boolean numpy mask of cells reachable from the
        # robot's current pose (via 4-connected value-0 cells), or None
        # if costmap/TF is not ready.
        if self.costmap_msg is None:
            return None
        grid = self.costmap_msg
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        arr = np.array(grid.data, dtype=np.int8).reshape(h, w)

        # Robot pose in map frame
        try:
            tf_robot = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return None
        rx = tf_robot.transform.translation.x
        ry = tf_robot.transform.translation.y

        # Convert to cell indices
        cx = int(round((rx - ox) / res))
        cy = int(round((ry - oy) / res))
        if not (0 <= cx < w and 0 <= cy < h):
            return None

        # If the exact robot cell is not value 0 (e.g. costmap inflation
        # makes it >0), search a small neighborhood for a free seed cell.
        if arr[cy, cx] != 0:
            seed = None
            for r in range(1, 6):
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w and arr[ny, nx] == 0:
                            seed = (ny, nx)
                            break
                    if seed:
                        break
                if seed:
                    break
            if seed is None:
                return None
            cy, cx = seed

        # 4-connected BFS over value-0 cells
        mask = np.zeros_like(arr, dtype=bool)
        q = deque()
        q.append((cy, cx))
        mask[cy, cx] = True
        while q:
            y, x = q.popleft()
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                ny, nx = y + dy, x + dx
                if (
                    0 <= ny < h and 0 <= nx < w
                    and not mask[ny, nx]
                    and arr[ny, nx] == 0
                ):
                    mask[ny, nx] = True
                    q.append((ny, nx))

        # Drop cells too close to the map boundary. Without this, sampled
        # waypoints near the edges make the global NavFn planner expand
        # its potential field over out-of-bounds pixels, spamming
        # "worldToMap failed: mx,my: ..., size_x,size_y: ..." tens of
        # thousands of times per plan attempt and slowing the whole
        # search to a crawl.
        margin_cells = int(math.ceil(EDGE_MARGIN / res))
        if margin_cells > 0:
            mask[:margin_cells, :] = False
            mask[-margin_cells:, :] = False
            mask[:, :margin_cells] = False
            mask[:, -margin_cells:] = False
        return mask

    def sample_random_xy(self):
        # Returns (x, y) in MAP frame or None if no usable cell is found.
        #
        # Strategy: distance-weighted sample from cells in the same free
        # component as the robot. Cells near the robot are exponentially
        # more likely than far ones (PREFERRED_RANGE = soft scale), and a
        # hard cap (SOFT_MAX_RANGE) excludes anything beyond ~3 m unless
        # nothing closer is unvisited. Visited cells stay rejected via
        # VISITED_RADIUS.
        #
        # Rationale (vs uniform sampling): with a slow Gazebo RTF, every
        # 7-8 m traverse eats the NAV_TIMEOUT before the robot can spin to
        # scan. Local-first sampling keeps each traverse short, so the
        # robot actually completes the 4-yaw spin at most waypoints.
        mask = self._compute_reachable_mask()
        if mask is None:
            return None
        free = np.argwhere(mask)
        if len(free) == 0:
            return None

        # Robot pose (for the distance weighting)
        try:
            tf_robot = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return None
        rx = tf_robot.transform.translation.x
        ry = tf_robot.transform.translation.y

        grid = self.costmap_msg
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        # Vectorize: every reachable cell -> (x, y) in map frame
        cxs = free[:, 1].astype(np.float64)
        cys = free[:, 0].astype(np.float64)
        xs = ox + (cxs + 0.5) * res
        ys = oy + (cys + 0.5) * res

        # Distance from robot
        dists = np.hypot(xs - rx, ys - ry)

        # Visited rejection (vectorized via broadcasting)
        if self.visited_waypoints:
            vx = np.array([p[0] for p in self.visited_waypoints])
            vy = np.array([p[1] for p in self.visited_waypoints])
            # (N, V) distance matrix -> min along V -> (N,)
            min_visited = np.min(
                np.hypot(xs[:, None] - vx[None, :], ys[:, None] - vy[None, :]),
                axis=1,
            )
            not_visited = min_visited >= VISITED_RADIUS
        else:
            not_visited = np.ones(len(xs), dtype=bool)

        # First try: cells within SOFT_MAX_RANGE and not visited.
        candidate = not_visited & (dists <= SOFT_MAX_RANGE)
        if not np.any(candidate):
            # No nearby unvisited cells -> drop the hard cap and accept any
            # unvisited cell. Still distance-weighted so we still prefer
            # closer ones, just not capped.
            candidate = not_visited
            if not np.any(candidate):
                return None  # area saturated -> caller resets memory

        cand_xs = xs[candidate]
        cand_ys = ys[candidate]
        cand_d = dists[candidate]

        # Weight = exp(-d / PREFERRED_RANGE). Cells at PREFERRED_RANGE are
        # ~37% as likely as cells at the robot's foot, etc.
        weights = np.exp(-cand_d / PREFERRED_RANGE)
        # Numerical safety
        wsum = weights.sum()
        if wsum <= 0.0 or not np.isfinite(wsum):
            return None
        probs = weights / wsum

        idx = int(np.random.choice(len(cand_xs), p=probs))
        return (float(cand_xs[idx]), float(cand_ys[idx]))

    # ------------------------------------------------------------------
    # Arm helpers (Lab4 non-blocking pattern)
    # ------------------------------------------------------------------
    def move_arm_to_home(self):
        self.get_logger().info(
            "Sending arm to HOME joint configuration + head to mild tilt down"
        )
        self.arm_motion_started = False
        self.arm_motion_done = False
        self.arm.move_to_configuration(
            joint_positions=HOME_JOINT_POSITIONS,
            joint_names=JOINT_ARM_NAMES,
        )
        #######################################################################
        # GROUP 30 - head pre-positioning bundled with arm tuck
        # Send the head trajectory in the same method call so the "go to
        # initial configuration" is a single user-visible operation.
        # Tilt = -0.5 rad (~17 deg below horizon) keeps ArUco markers on
        # cube faces / walls inside the camera FOV at 1-3 m without losing
        # the horizontal field where most markers sit.
        #######################################################################
        head_msg = JointTrajectory()
        head_msg.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.5]  # pan 0, tilt 17 deg down
        point.time_from_start = Duration(seconds=1.5).to_msg()
        head_msg.points.append(point)
        self.head_pub.publish(head_msg)
        #######################################################################
        # END GROUP 30
        #######################################################################

    def update_arm_flags(self):
        # The `not self.arm_motion_done` guard makes the "finished" log fire
        # exactly once per motion (otherwise this branch matches every loop
        # tick once IDLE is reached and spams at ~20 Hz).
        state = self.arm.query_state()
        if not self.arm_motion_started and state == MoveIt2State.EXECUTING:
            self.arm_motion_started = True
            self.get_logger().info("Arm motion started")
        if (
            self.arm_motion_started
            and not self.arm_motion_done
            and state == MoveIt2State.IDLE
        ):
            self.arm_motion_done = True
            self.get_logger().info("Arm motion finished")

    #######################################################################
    # GROUP 30 - Task 2 random spawn / AMCL global localization
    #
    # The exam requires that the robot starts at a random pose (different
    # from the SLAM mapping pose) and that the map<->odom alignment be
    # PERFORMED, not just told to AMCL. We follow the standard ROS 2 / Nav2
    # workflow described in the labs:
    #
    #   1) Call the AMCL service /reinitialize_global_localization
    #      (std_srvs/Empty). AMCL drops its current single-modal estimate
    #      and re-spreads its particles uniformly across the static map.
    #   2) Rotate the robot using the Nav2 "spin" action
    #      (nav2_msgs/action/Spin). A full 2*pi rotation gives AMCL laser
    #      scans from every heading, so particle weights converge fast.
    #   3) Monitor /amcl_pose (geometry_msgs/PoseWithCovarianceStamped):
    #      when the position and yaw covariance drop below the thresholds
    #      we consider the robot localized.
    #
    # No callbacks are registered on the futures -- we poll them in
    # update_spin_flags() each state-machine tick (same Lab 4 polling
    # pattern as update_arm_flags() / update_nav_flags()).
    #######################################################################
    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        # Lab-style: store latest message; convergence is checked in
        # update_amcl_flags() during the state-machine tick.
        self.amcl_pose = msg

    def request_global_localization(self) -> bool:
        # Service call: equivalent to running
        #   ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
        # AMCL replies after redistributing particles.
        if not self.global_loc_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "/reinitialize_global_localization service not available"
            )
            return False
        self.global_loc_client.call_async(Empty.Request())
        self.get_logger().info(
            "Requested AMCL /reinitialize_global_localization "
            "(uniform particle distribution)"
        )
        return True

    def clear_local_costmap(self) -> bool:
        # Service call equivalent to:
        #   ros2 service call /local_costmap/clear_entirely_local_costmap \
        #       nav2_msgs/srv/ClearEntireCostmap
        # Same as Nav2's internal "ClearLocalCostmap" recovery BT node.
        if not self.clear_local_costmap_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                "/local_costmap/clear_entirely_local_costmap service unavailable"
            )
            return False
        self.clear_local_costmap_client.call_async(ClearEntireCostmap.Request())
        self.get_logger().info("Cleared local costmap (stale obstacle markers)")
        return True

    def clear_global_costmap(self) -> bool:
        # Equivalent of clear_local_costmap on the global costmap. Called
        # once after AMCL converges to drop the phantom obstacle marks
        # accumulated under the wrong pre-convergence pose.
        if not self.clear_global_costmap_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                "/global_costmap/clear_entirely_global_costmap service unavailable"
            )
            return False
        self.clear_global_costmap_client.call_async(ClearEntireCostmap.Request())
        self.get_logger().info(
            "Cleared global costmap (phantom obstacles from pre-convergence pose)"
        )
        return True

    def send_localization_spin(self, target_yaw: float = 2.0 * math.pi):
        # Standard Nav2 Spin action -- same one used internally by the
        # recovery BT. Used here to rotate the robot in place so AMCL
        # sees the environment from every direction.
        self.spin_active = True
        self.spin_done = False
        self.spin_succeeded = False
        self._spin_send_future = None
        self._spin_goal_handle = None
        self._spin_result_future = None

        if not self.spin_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 /spin action server not available")
            self.spin_done = True
            self.spin_succeeded = False
            self.spin_active = False
            return

        goal_msg = Spin.Goal()
        goal_msg.target_yaw = target_yaw
        goal_msg.time_allowance = Duration(seconds=30.0).to_msg()
        self.get_logger().info(
            f"Sending Spin action goal: target_yaw={target_yaw:.2f} rad "
            f"({math.degrees(target_yaw):.1f} deg)"
        )
        self._spin_send_future = self.spin_client.send_goal_async(goal_msg)

    def update_spin_flags(self):
        # Polling counterpart to update_nav_flags() but for the Spin action.
        if self._spin_send_future is not None and self._spin_send_future.done():
            try:
                handle = self._spin_send_future.result()
            except Exception:
                handle = None
            self._spin_send_future = None
            if handle is None or not handle.accepted:
                self.get_logger().warn("Nav2 rejected the Spin goal")
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
            self.get_logger().info(
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
        if self.amcl_pose is None:
            return
        cov = self.amcl_pose.pose.covariance
        cov_xx = cov[0]
        cov_yy = cov[7]
        cov_yaw = cov[35]
        if cov_xx < 0.10 and cov_yy < 0.10 and cov_yaw < 0.07:
            if not self.amcl_converged:
                self.amcl_converged = True
                x = self.amcl_pose.pose.pose.position.x
                y = self.amcl_pose.pose.pose.position.y
                self.get_logger().info(
                    f"AMCL converged at map=({x:.2f}, {y:.2f}); "
                    f"cov_xx={cov_xx:.4f}, cov_yy={cov_yy:.4f}, "
                    f"cov_yaw={cov_yaw:.4f}"
                )
                #######################################################
                # GROUP 30 - clear global costmap on convergence latch
                # Done HERE rather than in State 2d so it fires once,
                # exactly at the convergence event, independently of
                # whether State 2d had already advanced (the "proceeding
                # anyway after 3 spins" branch had previously skipped
                # the clear, leaving phantom obstacles in the costmap).
                #######################################################
                self.clear_global_costmap()
                #######################################################
                # END GROUP 30
                #######################################################
    #######################################################################
    # END GROUP 30
    #######################################################################

    # ------------------------------------------------------------------
    # Nav2 helpers (non-blocking)
    # ------------------------------------------------------------------
    def send_nav_goal(self, x: float, y: float, yaw: float):
        # Reset flags + drop any stale futures from a previous goal.
        # We don't register done_callbacks, so old futures dropped here
        # simply stop being polled and have no further side-effects on
        # this node (same logic as resetting MoveIt2 flags before
        # arm.move_to_pose() in Lab 4).
        self.nav_goal_active = True
        self.nav_goal_done = False
        self.nav_goal_succeeded = False
        self._timeout_fired = False
        self._nav_goal_handle = None
        self._pending_send_future = None
        self._nav_result_future = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = yaw_to_quat(yaw)

        self.get_logger().info(
            f"Nav2 goal -> ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f} deg)"
        )

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            self.nav_goal_done = True
            self.nav_goal_succeeded = False
            return

        # Store the future. NO add_done_callback -- we poll it.
        self._pending_send_future = self.nav_client.send_goal_async(goal_msg)

    def update_nav_flags(self):
        # Lab 4-style polling of the Nav2 action futures. Called every
        # iteration of run_state_machine, next to update_arm_flags().
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
                self.get_logger().warn("Nav2 rejected the goal")
                self.nav_goal_done = True
                self.nav_goal_succeeded = False
                self.nav_goal_active = False
            else:
                self._nav_goal_handle = handle
                # Same idea: get the result future, store it, poll it.
                self._nav_result_future = handle.get_result_async()

        # Stage 2: did the RESULT future complete (goal succeeded/failed)?
        if (
            self._nav_result_future is not None
            and self._nav_result_future.done()
        ):
            try:
                status = self._nav_result_future.result().status
            except Exception:
                status = 5  # treat as CANCELED
            self._nav_result_future = None
            # status == 4 -> SUCCEEDED in action_msgs/GoalStatus
            self.nav_goal_succeeded = (status == 4)
            self.nav_goal_done = True
            self.nav_goal_active = False
            self.get_logger().info(
                f"Nav2 goal finished, status={status}, "
                f"succeeded={self.nav_goal_succeeded}"
            )

    def cancel_current_nav_goal(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()

    def _approach_pose_to_xy_yaw(self, ps: PoseStamped):
        x = ps.pose.position.x
        y = ps.pose.position.y
        yaw = quat_to_yaw(
            ps.pose.orientation.x,
            ps.pose.orientation.y,
            ps.pose.orientation.z,
            ps.pose.orientation.w,
        )
        return x, y, yaw

    # ------------------------------------------------------------------
    # Main state machine
    # ------------------------------------------------------------------
    def run_state_machine(self, executor_ready: Event):
        self.get_logger().info("State machine: waiting for executor...")
        executor_ready.wait()
        self.get_logger().info("Executor ready, giving the stack 10s to come up")
        time.sleep(10.0)

        PLANNING_TIMEOUT = 6.0
        # SEARCH_NAV_TIMEOUT: per-goal Nav2 timeout for State 3 search hops.
        # Kept short so unreachable cells (the random sampler can pick one
        # in an unmapped pocket) do not keep the robot stuck.
        # APPROACH_NAV_TIMEOUT: per-goal Nav2 timeout for State 4 (PICK) and
        # State 5 (PLACE), where the target is a known marker location that
        # can be several meters away. Slow Gazebo RTF means 25 s isn't enough
        # to drive 3+ m; 60 s gives the robot a realistic window.
        SEARCH_NAV_TIMEOUT = 25.0
        #######################################################################
        # GROUP 30 - APPROACH_NAV_TIMEOUT raised from 60 -> 180 s
        # PICK -> PLACE traversal can be ~5 m through tight pockets of the
        # group30 world. When DWB cannot make progress, the Nav2 BT
        # navigate_w_replanning_and_recovery branch runs a recovery cycle
        # (ClearLocalCostmap + Spin + Wait + replan), each cycle ~10-15 s.
        # At 60 s we were cancelling after only 2-3 cycles and re-sending
        # the same goal, restarting the BT from scratch and losing all
        # progress. 180 s lets the BT exhaust ~10 recovery cycles before
        # we give up and retry from State 5.
        #######################################################################
        APPROACH_NAV_TIMEOUT = 180.0
        # /spin 2*pi at max_rotational_vel=1.0 rad/s nominally takes ~6.3 s.
        # 15 s leaves margin for Gazebo RTF + behavior_server overhead.
        SEARCH_SPIN_TIMEOUT = 15.0
        send_time = None

        while rclpy.ok() and not self.finished:
            try:
                self.update_arm_flags()
                self.update_nav_flags()
                #######################################################
                # GROUP 30 - Task 2 random spawn / global localization
                # Spin futures and AMCL covariance are polled every tick.
                #######################################################
                self.update_spin_flags()
                self.update_amcl_flags()
                #######################################################
                # END GROUP 30
                #######################################################

                #######################################################################
                # GROUP 30 - REORDERED STATE MACHINE for Task 2 random spawn
                #
                #   State 0: tuck arm to HOME via MoveIt (was State 1).
                #            Done FIRST because the spawn pose has the arm
                #            extended, which is dangerous near obstacles
                #            and would block both the LiDAR / depth FOV
                #            and the eventual ArUco detection.
                #   State 1: wait for arm motion to finish (was State 2).
                #   State 2: AMCL global localization (was State 0).
                #            Sub-states:
                #              loc_substate==0  call /reinitialize_global_localization
                #              loc_substate==1  call /local_costmap/clear_entirely_local_costmap
                #                               (removes stale marks that would falsely
                #                                trip "Collision Ahead" on Spin)
                #              loc_substate==2  send Nav2 /spin goal of 2*pi
                #              loc_substate==3  wait for AMCL covariance to converge
                #   State 3..6: search / PICK / PLACE / done (unchanged).
                #######################################################################
                if self.state == 0:
                    self.get_logger().info("State 0: tuck arm to HOME (MoveIt)")
                    self.move_arm_to_home()
                    send_time = time.time()
                    self.state = 1

                elif self.state == 1:
                    if self.arm_motion_done:
                        self.get_logger().info(
                            "Arm at HOME, starting AMCL global localization"
                        )
                        self.state = 2
                    elif (
                        not self.arm_motion_started
                        and (time.time() - send_time) > PLANNING_TIMEOUT
                    ):
                        self.get_logger().warn("Arm planning timed out, retrying")
                        self.state = 0

                elif self.state == 2:
                    if self.loc_substate == 0:
                        self.get_logger().info(
                            "State 2a: requesting AMCL global localization"
                        )
                        if self.request_global_localization():
                            # Give AMCL a moment to redistribute particles
                            # before any further action.
                            time.sleep(2.0)
                            self.loc_substate = 1
                        else:
                            time.sleep(2.0)  # service unavailable -> retry
                    elif self.loc_substate == 1:
                        self.get_logger().info(
                            "State 2b: clearing local costmap before spin"
                        )
                        self.clear_local_costmap()
                        time.sleep(0.5)  # let the costmap process the clear
                        self.loc_substate = 2
                    elif self.loc_substate == 2:
                        self.localization_spin_count += 1
                        self.get_logger().info(
                            f"State 2c: spin attempt "
                            f"#{self.localization_spin_count} (target_yaw=2*pi)"
                        )
                        self.send_localization_spin(2.0 * math.pi)
                        send_time = time.time()
                        self.loc_substate = 3
                    elif self.loc_substate == 3:
                        # Convergence wins over spin completion: as soon as
                        # AMCL is confident we can move on, even mid-spin.
                        if self.amcl_converged:
                            self.get_logger().info(
                                "State 2d: AMCL aligned -- proceeding to search"
                            )
                            if self.spin_active and self._spin_goal_handle is not None:
                                self._spin_goal_handle.cancel_goal_async()
                            # Note: clear_global_costmap() is now fired
                            # automatically inside update_amcl_flags() on
                            # the convergence transition, so it's already
                            # been called by this point (whether AMCL
                            # converged in this state or later).
                            self.state = 3
                            continue
                        if self.spin_done:
                            # Spin finished but AMCL not yet converged.
                            if self.localization_spin_count >= 3:
                                self.get_logger().warn(
                                    f"AMCL not converged after "
                                    f"{self.localization_spin_count} spins, "
                                    "proceeding anyway"
                                )
                                self.state = 3
                                continue
                            self.get_logger().info(
                                "Spin done, AMCL not converged yet -- "
                                "clearing costmap and re-spinning"
                            )
                            self.spin_done = False
                            self.loc_substate = 1  # back to clear costmap
                #######################################################################
                # END GROUP 30
                #######################################################################

                # ---- State 3: random search w/ visited memory + 4-yaw spin
                elif self.state == 3:
                    # Exit condition: both approach poses latched.
                    if (
                        self.pick_approach_pose is not None
                        and self.place_approach_pose is not None
                    ):
                        if self.nav_goal_active:
                            self.get_logger().info(
                                "Both markers seen mid-search, cancelling nav"
                            )
                            self.cancel_current_nav_goal()
                        if self.spin_active and self._spin_goal_handle is not None:
                            self.get_logger().info(
                                "Both markers seen mid-spin, cancelling spin"
                            )
                            self._spin_goal_handle.cancel_goal_async()
                        # Clear ALL goal state so a stale callback from the
                        # cancelled in-flight goal doesn't bleed into State 4.
                        self.nav_goal_active = False
                        self.nav_goal_done = False
                        self.nav_goal_succeeded = False
                        self.spin_active = False
                        self.spin_done = False
                        self.spin_succeeded = False
                        self.search_phase = 'sample'
                        # Freeze the approach poses now so Nav2's target
                        # doesn't keep drifting during the final approach.
                        self._refresh_approach_poses = False
                        self.get_logger().info(
                            f"Freezing PICK approach at "
                            f"({self.pick_approach_pose.pose.position.x:.2f}, "
                            f"{self.pick_approach_pose.pose.position.y:.2f}) and "
                            f"PLACE approach at "
                            f"({self.place_approach_pose.pose.position.x:.2f}, "
                            f"{self.place_approach_pose.pose.position.y:.2f})"
                        )
                        self.state = 4
                        continue

                    #######################################################
                    # GROUP 30 - waypoint phase machine
                    #######################################################
                    # 1) Handle nav goal completion (only relevant in 'nav')
                    if self.search_phase == 'nav' and self.nav_goal_done:
                        nav_ok = self.nav_goal_succeeded
                        self.nav_goal_done = False
                        if nav_ok:
                            # Arrived at waypoint -> launch in-place Spin 2*pi
                            self.get_logger().info(
                                "Waypoint reached, sending Spin 2*pi"
                            )
                            self.send_localization_spin(2.0 * math.pi)
                            self.search_phase = 'spin'
                            send_time = time.time()
                        else:
                            # Nav failed -> abandon this waypoint
                            self.get_logger().warn(
                                "Waypoint nav failed -> sampling next"
                            )
                            self.search_phase = 'sample'

                    # 2) Handle spin completion (only relevant in 'spin')
                    if self.search_phase == 'spin' and self.spin_done:
                        spin_ok = self.spin_succeeded
                        self.spin_done = False
                        if not spin_ok:
                            self.get_logger().warn(
                                "Waypoint spin aborted -> sampling next"
                            )
                        # Either way: done with this waypoint, go sample next
                        self.search_phase = 'sample'

                    # 3) Watch nav timeout
                    if self.search_phase == 'nav' and self.nav_goal_active:
                        if (
                            not self._timeout_fired
                            and (time.time() - send_time) > SEARCH_NAV_TIMEOUT
                        ):
                            self.get_logger().warn(
                                "Nav goal timed out, cancelling -> sample next"
                            )
                            self.cancel_current_nav_goal()
                            self._timeout_fired = True
                        continue

                    # 4) Watch spin timeout
                    if self.search_phase == 'spin' and self.spin_active:
                        if (
                            not self._timeout_fired
                            and (time.time() - send_time) > SEARCH_SPIN_TIMEOUT
                        ):
                            self.get_logger().warn(
                                "Spin timed out, cancelling -> sample next"
                            )
                            if self._spin_goal_handle is not None:
                                self._spin_goal_handle.cancel_goal_async()
                            self._timeout_fired = True
                        continue

                    # 5) Idle ('sample') -> draw a new waypoint and send nav
                    if self.search_phase == 'sample':
                        if self.costmap_msg is None:
                            self.get_logger().info(
                                "Waiting for /global_costmap/costmap...",
                                throttle_duration_sec=2.0,
                            )
                            time.sleep(0.5)
                            continue
                        xy = self.sample_random_xy()
                        if xy is None:
                            # sample_random_xy returns None for either
                            # (a) the visited set covers all reachable cells
                            #     -> real saturation, clearing helps;
                            # (b) costmap / TF lookup was transiently
                            #     unavailable -> clearing visited does
                            #     nothing useful, we should just wait.
                            self.get_logger().warn(
                                "Sample returned None, clearing visited "
                                "list and retrying",
                                throttle_duration_sec=2.0,
                            )
                            self.visited_waypoints = []
                            time.sleep(0.5)
                            continue
                        x, y = xy
                        self.visited_waypoints.append((x, y))
                        self.get_logger().info(
                            f"New random waypoint ({x:.2f}, {y:.2f}); "
                            f"visited={len(self.visited_waypoints)}"
                        )
                        self.send_nav_goal(x, y, 0.0)
                        self.search_phase = 'nav'
                        send_time = time.time()
                    #######################################################
                    # END GROUP 30
                    #######################################################

                # ---- State 4: navigate to PICK approach
                elif self.state == 4:
                    # 1) Handle goal completion FIRST (see State 3 comment).
                    if self.nav_goal_done:
                        self.nav_goal_done = False  # consume
                        if self.nav_goal_succeeded:
                            self.get_logger().info(
                                "PICK approach reached, advancing to PLACE"
                            )
                            self.state = 5
                            continue
                        else:
                            self.get_logger().warn(
                                "PICK nav did not succeed, retrying"
                            )
                            # fall through -> the (not active) branch below
                            # will re-send the goal.

                    # 2) In flight -> only watch for timeout.
                    if self.nav_goal_active:
                        if (
                            not self._timeout_fired
                            and (time.time() - send_time) > APPROACH_NAV_TIMEOUT
                        ):
                            self.get_logger().warn(
                                "PICK nav timed out, cancelling for retry"
                            )
                            self.cancel_current_nav_goal()
                            self._timeout_fired = True
                        continue

                    # 3) Not active -> send (initial or retry).
                    x, y, yaw = self._approach_pose_to_xy_yaw(
                        self.pick_approach_pose
                    )
                    self.get_logger().info("State 4: nav to PICK approach")
                    self.send_nav_goal(x, y, yaw)
                    send_time = time.time()

                # ---- State 5: navigate to PLACE approach
                elif self.state == 5:
                    # 1) Handle goal completion FIRST (see State 3 comment).
                    if self.nav_goal_done:
                        self.nav_goal_done = False  # consume
                        if self.nav_goal_succeeded:
                            self.get_logger().info(
                                "PLACE approach reached, task done"
                            )
                            self.state = 6
                            continue
                        else:
                            self.get_logger().warn(
                                "PLACE nav did not succeed, retrying"
                            )

                    # 2) In flight -> only watch for timeout.
                    if self.nav_goal_active:
                        if (
                            not self._timeout_fired
                            and (time.time() - send_time) > APPROACH_NAV_TIMEOUT
                        ):
                            self.get_logger().warn(
                                "PLACE nav timed out, cancelling for retry"
                            )
                            self.cancel_current_nav_goal()
                            self._timeout_fired = True
                        continue

                    # 3) Not active -> send (initial or retry).
                    x, y, yaw = self._approach_pose_to_xy_yaw(
                        self.place_approach_pose
                    )
                    self.get_logger().info("State 5: nav to PLACE approach")
                    self.send_nav_goal(x, y, yaw)
                    send_time = time.time()

                # ---- State 6: done
                elif self.state == 6:
                    self.get_logger().info("State 6: Task 2 completed")
                    self.finished = True
                    break

            except Exception as e:
                import traceback
                self.get_logger().error(
                    f"Exception in state machine: {e}\n{traceback.format_exc()}"
                )

            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = Task2Manager()

    executor_ready = Event()
    state_thread = Thread(
        target=node.run_state_machine, args=(executor_ready,), daemon=True
    )
    state_thread.start()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_ready.set()

    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
