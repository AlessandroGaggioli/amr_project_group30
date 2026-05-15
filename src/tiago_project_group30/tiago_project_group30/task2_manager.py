#!/usr/bin/env python3
# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# Non-blocking state machine (modeled on Lab4 state_machine_not_blocking.py)
# that fulfills the Task 2 specification:
#   1) Randomizes the initial pose: teleports the robot in Gazebo via
#      /set_entity_state and publishes the matching pose on /initialpose so
#      AMCL aligns the saved map with the navigation system.
#   2) Tucks the arm to a safe HOME configuration via MoveIt2
#      (same pattern as task1_manager.py).
#   3) Autonomously visits a small set of look-around waypoints with Nav2
#      while the two aruco_single detectors run in background. The first
#      time each marker (ID 26 = pick, ID 238 = place) is seen, its pose
#      in the map frame is latched via TF.
#   4) Once both markers are localized, drives to an approach pose in front
#      of the pick location and then in front of the place location.

import math
import random
import time
from threading import Thread, Event

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
)
from nav2_msgs.action import NavigateToPose
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from pymoveit2 import MoveIt2, MoveIt2State

from PyKDL import Rotation


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

# Same HOME configuration used in task1_manager: torso lifted + arm tucked
# so the robot can navigate safely without colliding with itself.
HOME_JOINT_POSITIONS = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]

##############################
# Map reference for the random spawn
##############################

# During Task 1 (SLAM) the robot started at Gazebo world (0.0, -1.3, 0.0).
# The `map` frame is anchored to that starting pose, so the conversion is:
#     map_pose = (x_w - 0.0, y_w - (-1.3), yaw) = (x_w, y_w + 1.3, yaw)
# i.e. the map frame is located at world (0.0, -1.3, 0.0).
MAP_ORIGIN_IN_WORLD = (0.0, -1.3, 0.0)

# Candidate random initial poses (in the MAP frame, in meters/rad).
# All chosen to fall in free space inside the group30 environment.
# Tune these if any of them ends up inside an obstacle in your specific world.
RANDOM_MAP_POSES = [
    (0.0, 0.0, 0.0),
    (0.5, 0.3, math.radians(30)),
    (-0.5, 0.4, math.radians(-45)),
    (0.7, -0.2, math.radians(90)),
    (-0.3, -0.4, math.radians(180)),
]

##############################
# Discovery waypoints
##############################

# Look-around waypoints (in the MAP frame). The robot visits each in order
# while the ArUco detectors scan the environment. As soon as BOTH markers
# have been seen, the state machine aborts the search and navigates to the
# pick / place approach poses.
SEARCH_WAYPOINTS = [
    (1.5, 0.0, math.radians(0)),
    (1.5, 0.0, math.radians(180)),
    (-1.0, 1.0, math.radians(-90)),
    (-1.5, -1.5, math.radians(45)),
    (2.0, -1.5, math.radians(135)),
]

##############################
# ArUco / approach parameters
##############################

# TF frame names published by the two aruco_single nodes launched in
# task2.launch.py. Must match the marker_frame parameters there.
PICK_MARKER_FRAME = "aruco_pick_frame"     # ID  26
PLACE_MARKER_FRAME = "aruco_place_frame"   # ID 238

# Distance the robot stops in front of a marker (along the marker's local
# +Z axis, which points outward from the cube face by aruco_ros convention).
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


##############################


class Task2Manager(Node):

    def __init__(self):
        super().__init__("task2_manager")

        self.robot_base_frame = "base_link"
        self.map_frame = "map"

        # ---------- state machine state ----------
        self.state = 0
        self.finished = False

        # Arm motion bookkeeping (same pattern as Lab4 non-blocking SM)
        self.arm_motion_started = False
        self.arm_motion_done = False

        # Navigation bookkeeping
        self.nav_goal_active = False
        self.nav_goal_done = False
        self.nav_goal_succeeded = False
        self._nav_goal_handle = None

        # Marker poses latched the first time each is observed
        self.pick_marker_map_pose = None     # PoseStamped in map frame
        self.place_marker_map_pose = None

        # Diagnostics: True once the aruco_single node has *ever* published
        # a transform for that marker (independent of TF lookups).
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

        # ---------- TF (used to look up marker -> map) ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- Gazebo /set_entity_state client (random spawn) ----------
        self.set_entity_state_cli = self.create_client(
            SetEntityState, "/set_entity_state", callback_group=cb_group_io
        )

        # ---------- AMCL /initialpose publisher ----------
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # ---------- Nav2 action client ----------
        self.nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=cb_group_nav
        )

        # ---------- TF-polling timer for marker discovery ----------
        # The aruco_single node already publishes each detected marker as
        # a TF, so to get its pose for navigation we just look it up in
        # `map` (one shot, no manual frame composition).
        self.marker_poll_timer = self.create_timer(
            0.5, self.poll_markers, callback_group=cb_group_io
        )

        # ---------- Diagnostic subscriptions ----------
        # aruco_single also publishes a /transform topic (in the camera
        # optical frame). We subscribe to it ONLY as a diagnostic so we can
        # tell "marker never seen" apart from "marker seen but TF was not
        # lookable yet".
        self.create_subscription(
            TransformStamped,
            '/aruco_pick/transform',
            self._aruco_pick_diag_cb,
            10,
            callback_group=cb_group_io,
        )
        self.create_subscription(
            TransformStamped,
            '/aruco_place/transform',
            self._aruco_place_diag_cb,
            10,
            callback_group=cb_group_io,
        )

    def _aruco_pick_diag_cb(self, _msg):
        if not self.pick_seen_in_camera:
            self.pick_seen_in_camera = True
            self.get_logger().info(
                "[diag] aruco_single PICK (ID 26) IS seeing the marker"
            )

    def _aruco_place_diag_cb(self, _msg):
        if not self.place_seen_in_camera:
            self.place_seen_in_camera = True
            self.get_logger().info(
                "[diag] aruco_single PLACE (ID 238) IS seeing the marker"
            )

    # ------------------------------------------------------------------
    # Arm helpers (non-blocking pattern from Lab4)
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
        # Poll MoveIt2 state; flip flags exactly like state_machine_not_blocking.
        # The `not self.arm_motion_done` guard makes the "finished" log fire
        # exactly once per motion (the flag stays True forever otherwise, so
        # without the guard this branch matched on every iteration and the
        # log spammed at ~20 Hz).
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
    # Random initial pose: teleport in Gazebo + AMCL /initialpose
    # ------------------------------------------------------------------
    def randomize_initial_pose(self) -> bool:
        x_map, y_map, yaw = random.choice(RANDOM_MAP_POSES)
        x_world = x_map + MAP_ORIGIN_IN_WORLD[0]
        y_world = y_map + MAP_ORIGIN_IN_WORLD[1]

        self.get_logger().info(
            f"Randomizing initial pose -> map=({x_map:.2f}, {y_map:.2f}, "
            f"{math.degrees(yaw):.1f} deg), world=({x_world:.2f}, {y_world:.2f})"
        )

        # 1) Teleport the robot in Gazebo via /set_entity_state.
        # NOTE: this service is provided by the gazebo_ros_state plugin.
        # If the plugin is not loaded in the world SDF the service does NOT
        # exist and we fall back to "no teleport". In that case the robot
        # stays at the spawn position (0, -1.3) but we still publish the
        # random /initialpose to AMCL -- which is a BAD outcome because
        # Gazebo and AMCL will disagree and navigation will misbehave.
        # Therefore: if teleport fails, we ALSO publish /initialpose at
        # the actual spawn position so at least everything is consistent.
        teleported = False
        if self.set_entity_state_cli.wait_for_service(timeout_sec=10.0):
            req = SetEntityState.Request()
            state = EntityState()
            state.name = "tiago"
            state.pose.position.x = x_world
            state.pose.position.y = y_world
            state.pose.position.z = 0.0
            state.pose.orientation = yaw_to_quat(yaw)
            state.reference_frame = "world"
            req.state = state

            future = self.set_entity_state_cli.call_async(req)
            t0 = time.time()
            while not future.done() and (time.time() - t0) < 5.0:
                time.sleep(0.05)
            if (
                future.done()
                and future.result() is not None
                and future.result().success
            ):
                teleported = True
            else:
                self.get_logger().warn("set_entity_state call did not succeed")
        else:
            self.get_logger().error(
                "/set_entity_state service NOT available. The "
                "gazebo_ros_state plugin is not loaded in this world. "
                "Falling back to: no teleport, AMCL initialpose set to "
                "the default spawn (0, 0, 0) in map frame."
            )

        if not teleported:
            # Keep Gazebo and AMCL consistent: tell AMCL the robot is at
            # the spawn. This avoids the map/laser mismatch that breaks
            # the local planner ("No valid trajectories...").
            x_map, y_map, yaw = 0.0, 0.0, 0.0

        # Let Gazebo physics settle before publishing /initialpose
        time.sleep(1.0)

        # 2) Publish /initialpose so AMCL knows where the robot is on the map.
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x_map
        msg.pose.pose.position.y = y_map
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        cov = [0.0] * 36
        cov[0] = 0.25      # x
        cov[7] = 0.25      # y
        cov[35] = 0.07     # yaw
        msg.pose.covariance = cov
        # Publish a few times: AMCL may not be subscribed yet at first.
        for _ in range(5):
            self.initialpose_pub.publish(msg)
            time.sleep(0.2)
        return True

    # ------------------------------------------------------------------
    # Marker polling: lookup pick / place TFs in the map frame
    # ------------------------------------------------------------------
    def poll_markers(self):
        # Latch each marker pose only the first time we manage to look it
        # up. This avoids overwriting a clean estimate with a noisier one.
        if self.pick_marker_map_pose is None:
            ps = self._lookup_marker_pose(PICK_MARKER_FRAME)
            if ps is not None:
                self.pick_marker_map_pose = ps
                self.get_logger().info(
                    f"PICK marker (ID 26) localized at map "
                    f"({ps.pose.position.x:.2f}, {ps.pose.position.y:.2f})"
                )

        if self.place_marker_map_pose is None:
            ps = self._lookup_marker_pose(PLACE_MARKER_FRAME)
            if ps is not None:
                self.place_marker_map_pose = ps
                self.get_logger().info(
                    f"PLACE marker (ID 238) localized at map "
                    f"({ps.pose.position.x:.2f}, {ps.pose.position.y:.2f})"
                )

    def _lookup_marker_pose(self, marker_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                marker_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return None
        ps = PoseStamped()
        ps.header.stamp = tf.header.stamp
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        return ps

    # ------------------------------------------------------------------
    # Navigation helpers (Nav2 NavigateToPose action, non-blocking)
    # ------------------------------------------------------------------
    def send_nav_goal(self, x: float, y: float, yaw: float):
        self.nav_goal_active = True
        self.nav_goal_done = False
        self.nav_goal_succeeded = False

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

        send_fut = self.nav_client.send_goal_async(goal_msg)
        send_fut.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal")
            self.nav_goal_done = True
            self.nav_goal_succeeded = False
            return
        self._nav_goal_handle = handle
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result()
        # status == 4 -> SUCCEEDED in action_msgs/GoalStatus
        self.nav_goal_succeeded = (result.status == 4)
        self.nav_goal_done = True
        self.nav_goal_active = False
        self.get_logger().info(
            f"Nav2 goal finished, status={result.status}, "
            f"succeeded={self.nav_goal_succeeded}"
        )

    def cancel_current_nav_goal(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()

    # Convert a marker pose (map frame) into a robot approach pose:
    # stand APPROACH_DISTANCE in front of the marker (along marker +Z) and
    # face the marker. Returns (x, y, yaw) in the map frame.
    def _marker_to_approach_pose(self, marker_pose: PoseStamped):
        q = marker_pose.pose.orientation
        rot = Rotation.Quaternion(q.x, q.y, q.z, q.w)
        # Direction of the marker's local +Z axis expressed in the map frame.
        zx, zy, _ = rot.UnitZ()
        # Approach point: marker_pos + APPROACH_DISTANCE * marker_+Z
        ax = marker_pose.pose.position.x + APPROACH_DISTANCE * zx
        ay = marker_pose.pose.position.y + APPROACH_DISTANCE * zy
        # Robot should face back toward the marker (opposite of +Z).
        yaw = math.atan2(-zy, -zx)
        return ax, ay, yaw

    # ------------------------------------------------------------------
    # Main state machine (non-blocking, ~20 Hz)
    # ------------------------------------------------------------------
    def run_state_machine(self, executor_ready: Event):
        self.get_logger().info("State machine: waiting for executor...")
        executor_ready.wait()
        self.get_logger().info("Executor ready, giving the stack 10s to come up")
        time.sleep(10.0)

        waypoint_idx = 0
        PLANNING_TIMEOUT = 6.0      # MoveIt planning timeout
        NAV_TIMEOUT = 60.0           # per-waypoint nav timeout
        send_time = None

        while rclpy.ok() and not self.finished:
            try:
                self.update_arm_flags()

                # ---- State 0: random teleport + AMCL initialpose --------
                if self.state == 0:
                    self.get_logger().info("State 0: random spawn + initialpose")
                    self.randomize_initial_pose()
                    # Give AMCL a moment to converge before doing anything else
                    time.sleep(3.0)
                    self.state = 1

                # ---- State 1: tuck arm via MoveIt -----------------------
                elif self.state == 1:
                    self.get_logger().info("State 1: tuck arm to HOME (MoveIt)")
                    self.move_arm_to_home()
                    send_time = time.time()
                    self.state = 2

                # ---- State 2: wait for arm to reach HOME ---------------
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

                # ---- State 3: drive to search waypoints ----------------
                elif self.state == 3:
                    # Shortcut: if both markers have already been spotted,
                    # cancel the current goal and jump to PICK approach.
                    if (
                        self.pick_marker_map_pose is not None
                        and self.place_marker_map_pose is not None
                    ):
                        if self.nav_goal_active:
                            self.get_logger().info(
                                "Both markers seen mid-search, cancelling nav"
                            )
                            self.cancel_current_nav_goal()
                        self.nav_goal_active = False
                        self.state = 4
                        continue

                    if not self.nav_goal_active:
                        if waypoint_idx >= len(SEARCH_WAYPOINTS):
                            self.get_logger().warn(
                                "Exhausted search waypoints, looping"
                            )
                            waypoint_idx = 0
                        x, y, yaw = SEARCH_WAYPOINTS[waypoint_idx]
                        waypoint_idx += 1
                        self.send_nav_goal(x, y, yaw)
                        send_time = time.time()
                    else:
                        if self.nav_goal_done:
                            self.nav_goal_active = False
                        elif (time.time() - send_time) > NAV_TIMEOUT:
                            self.get_logger().warn(
                                "Search nav timed out, next waypoint"
                            )
                            self.cancel_current_nav_goal()
                            self.nav_goal_active = False

                # ---- State 4: navigate to PICK approach ---------------
                elif self.state == 4:
                    if not self.nav_goal_active:
                        x, y, yaw = self._marker_to_approach_pose(
                            self.pick_marker_map_pose
                        )
                        self.get_logger().info("State 4: nav to PICK approach")
                        self.send_nav_goal(x, y, yaw)
                        send_time = time.time()
                    else:
                        if self.nav_goal_done:
                            self.nav_goal_active = False
                            self.state = 5
                        elif (time.time() - send_time) > NAV_TIMEOUT:
                            self.get_logger().warn("PICK nav timed out, retrying")
                            self.cancel_current_nav_goal()
                            self.nav_goal_active = False

                # ---- State 5: navigate to PLACE approach --------------
                elif self.state == 5:
                    if not self.nav_goal_active:
                        x, y, yaw = self._marker_to_approach_pose(
                            self.place_marker_map_pose
                        )
                        self.get_logger().info("State 5: nav to PLACE approach")
                        self.send_nav_goal(x, y, yaw)
                        send_time = time.time()
                    else:
                        if self.nav_goal_done:
                            self.nav_goal_active = False
                            self.state = 6
                        elif (time.time() - send_time) > NAV_TIMEOUT:
                            self.get_logger().warn("PLACE nav timed out, retrying")
                            self.cancel_current_nav_goal()
                            self.nav_goal_active = False

                # ---- State 6: done ------------------------------------
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
