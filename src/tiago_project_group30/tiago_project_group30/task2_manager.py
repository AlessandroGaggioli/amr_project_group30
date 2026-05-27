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
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

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
VISITED_RADIUS = 2.0  # meters

# Sampling is BIASED toward cells close to the robot's current position.
# - PREFERRED_RANGE: cells beyond this get exponentially less likely.
# - SOFT_MAX_RANGE: hard cap; cells beyond this are excluded entirely
#   unless no closer cell is available (then the cap is dropped).
# This produces "expanding ring" exploration: the robot first surveys
# nearby cells, only jumping farther once close cells are visited.
PREFERRED_RANGE = 1.5  # meters
SOFT_MAX_RANGE = 3.0   # meters

# Don't sample cells closer than this many meters to the saved-map edge.
# The global NavFn planner expands a potential field around the goal that
# reaches a few inflation radii outward; if that expansion crosses the map
# boundary it logs "worldToMap failed: mx,my: ..., size_x,size_y: ..." once
# *per iteration* and we get tens of thousands of errors per plan attempt.
# 0.6 m is comfortably more than the inflation radius (0.45 m) used in
# tiago_nav2.yaml, so the planner never reaches out-of-bounds pixels.
EDGE_MARGIN = 0.6  # meters

# At each sampled (x, y) the robot does this sequence of yaws so the head
# camera scans the surroundings.
# 3 yaws at 120 deg apart cover ~190 deg of camera FOV per waypoint
# (63 deg FOV + the angular sweep of the in-place rotation). This catches
# markers tucked at the side angles that the 2-yaw [0, pi] config was
# blind to, while still being ~25% faster than the full 4-yaw spin.
WAYPOINT_YAW_STEPS = [0.0, 2.0 * math.pi / 3.0, -2.0 * math.pi / 3.0]

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
APPROACH_DISTANCE = 0.7  # meters

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

        # ---------- Nav2 action client ----------
        self.nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=cb_group_nav
        )

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
        # waypoint_queue is the active 4-yaw spin sequence at the current
        # sampled (x, y). When empty we draw a new sample.
        self.waypoint_queue = []

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
        # KEEP-CLOSEST policy: only overwrite the stored marker_in_map if
        # this new observation is closer than the previous one.
        new_distance = math.sqrt(
            aruco_in_cam.p.x() ** 2
            + aruco_in_cam.p.y() ** 2
            + aruco_in_cam.p.z() ** 2
        )
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
    # Approach-frame broadcaster (lab3-style)
    # ------------------------------------------------------------------
    # The marker pose in MAP frame is already stored stably by the
    # subscription callback (see _handle_aruco_detection). Here we just
    # compute the approach offset and broadcast the TF -- no per-tick
    # TF lookup, no recomposition.
    def publish_approach_frames(self):
        if self.pick_marker_in_map is not None:
            approach = self.pick_marker_in_map * Frame(
                Rotation(), Vector(0, 0, APPROACH_DISTANCE)
            )
            # Same rotation lab3 applies so the frame's X axis points back
            # toward the marker (i.e. base-forward will face the marker).
            approach.M.DoRotY(math.pi / 2.0)
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
            approach = self.place_marker_in_map * Frame(
                Rotation(), Vector(0, 0, APPROACH_DISTANCE)
            )
            approach.M.DoRotY(math.pi / 2.0)
            self._broadcast_frame(approach, PLACE_APPROACH_FRAME)
            first_time = self.place_approach_pose is None
            if first_time or self._refresh_approach_poses:
                self.place_approach_pose = self._frame_to_pose_stamped(approach)
                if first_time:
                    self.get_logger().info(
                        f"PLACE approach localized at map "
                        f"({approach.p.x():.2f}, {approach.p.y():.2f})"
                    )

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
        self.get_logger().info("Sending arm to HOME joint configuration")
        self.arm_motion_started = False
        self.arm_motion_done = False
        self.arm.move_to_configuration(
            joint_positions=HOME_JOINT_POSITIONS,
            joint_names=JOINT_ARM_NAMES,
        )

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

    # ------------------------------------------------------------------
    # AMCL seeding (no random spawn for now -- deferred)
    # ------------------------------------------------------------------
    def set_initial_pose(self) -> bool:
        self.get_logger().info(
            "Seeding AMCL /initialpose at spawn (map 0, 0, 0) -- "
            "random spawn disabled for now"
        )
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation = yaw_to_quat(0.0)
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.07
        msg.pose.covariance = cov
        for _ in range(5):
            self.initialpose_pub.publish(msg)
            time.sleep(0.2)
        return True

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
        APPROACH_NAV_TIMEOUT = 60.0
        send_time = None

        while rclpy.ok() and not self.finished:
            try:
                self.update_arm_flags()
                self.update_nav_flags()

                # ---- State 0: seed AMCL at spawn (random spawn deferred)
                if self.state == 0:
                    self.get_logger().info("State 0: seed AMCL initialpose")
                    self.set_initial_pose()
                    time.sleep(3.0)
                    self.state = 1

                # ---- State 1: tuck arm via MoveIt
                elif self.state == 1:
                    self.get_logger().info("State 1: tuck arm to HOME (MoveIt)")
                    self.move_arm_to_home()
                    send_time = time.time()
                    self.state = 2

                # ---- State 2: wait for arm
                elif self.state == 2:
                    if self.arm_motion_done:
                        self.get_logger().info("Arm at HOME, starting search")
                        self.state = 3
                    elif (
                        not self.arm_motion_started
                        and (time.time() - send_time) > PLANNING_TIMEOUT
                    ):
                        self.get_logger().warn("Arm planning timed out, retrying")
                        self.state = 1

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
                        # Clear ALL goal state so a stale callback from the
                        # cancelled in-flight goal doesn't bleed into State 4.
                        self.nav_goal_active = False
                        self.nav_goal_done = False
                        self.nav_goal_succeeded = False
                        self.waypoint_queue = []
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

                    # 1) Handle goal completion FIRST. The result callback
                    # atomically sets active=False AND done=True, so if we
                    # check 'done' only inside the 'else' (active=True)
                    # branch we'd never see it -- which is exactly the bug
                    # we just hit.
                    if self.nav_goal_done:
                        if not self.nav_goal_succeeded:
                            # Fast-fail: Nav2 aborted. Drop the rest of the
                            # 4-yaw spin and try a fresh (x, y).
                            self.get_logger().warn(
                                "Nav2 goal failed -> dropping rest of spin"
                            )
                            self.waypoint_queue = []
                        self.nav_goal_done = False  # consume

                    # 2) If a goal is in flight, only watch for timeout.
                    if self.nav_goal_active:
                        if (
                            not self._timeout_fired
                            and (time.time() - send_time) > SEARCH_NAV_TIMEOUT
                        ):
                            self.get_logger().warn(
                                "Nav goal timed out, dropping rest of spin"
                            )
                            self.cancel_current_nav_goal()
                            self.waypoint_queue = []
                            self._timeout_fired = True
                        continue

                    # 3) No goal in flight -> need to send one.
                    if not self.waypoint_queue:
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
                            # Either way, throttle the log (would spam
                            # ~20 Hz without the sleep) and give the
                            # executor a moment before the next attempt.
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
                        self.waypoint_queue = [
                            (x, y, yaw) for yaw in WAYPOINT_YAW_STEPS
                        ]
                        self.get_logger().info(
                            f"New random waypoint ({x:.2f}, {y:.2f}); "
                            f"visited={len(self.visited_waypoints)}"
                        )

                    # Pop next yaw step from the spin queue
                    x, y, yaw = self.waypoint_queue.pop(0)
                    self.send_nav_goal(x, y, yaw)
                    send_time = time.time()

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
