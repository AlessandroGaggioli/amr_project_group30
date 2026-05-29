# TASK 3 of Autonomous Mobile Robotics Exam - Group 30
#
# Cube ArUco tracker. Same pattern as task2_aruco.ArucoTracker but tuned
# for the markers SITTING ON TOP of the two pick cubes (IDs 63 and 582).
# Differences vs. the wall-marker tracker:
#
#   - Detects only one marker AT A TIME (the cube currently being
#     handled). We keep last-known map-frame poses for both markers
#     anyway, in case the camera sees ID 582 while we're already in
#     Task 3 for ID 63.
#   - Does NOT compute / broadcast an approach frame: the base is
#     already at the (Task 2) PICK approach pose -- we use the cube
#     marker only to derive the GRASP pose for the arm.
#   - Composes the marker_in_camera into the map frame at the time of
#     detection (msg.header.stamp), same trick task2_aruco uses to avoid
#     stale-TF drift.

import math

import rclpy.time
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
from PyKDL import Frame, Rotation, Vector

from tiago_project_group30.task2_constants import (
    CAMERA_FRAME,
    MAX_DETECTION_DISTANCE,
)
from tiago_project_group30.task2_kdl_helpers import transform_to_kdl_frame
from tiago_project_group30.task3_constants import (
    CUBE_ARUCO_TOPICS,
    CUBE_PICK_SEQUENCE,
)


class CubeTracker:

    def __init__(self, node, tf_buffer, amcl, cb_group_io,
                 map_frame: str = "map"):
        self.node = node
        self.tf_buffer = tf_buffer
        self.amcl = amcl
        self.map_frame = map_frame
        self.camera_frame = CAMERA_FRAME

        # Per-cube map-frame pose, closest-observation policy (same as
        # task2_aruco): keep the observation taken from closest range
        # because aruco PnP error grows with distance.
        self.cube_marker_in_map = {cid: None for cid in CUBE_PICK_SEQUENCE}
        self.cube_detection_distance = {
            cid: float("inf") for cid in CUBE_PICK_SEQUENCE
        }
        self.cube_seen_in_camera = {cid: False for cid in CUBE_PICK_SEQUENCE}

        # Master gate: during Task 2 (random search) the camera may
        # briefly glimpse a cube marker from far away at an oblique
        # angle, producing a noisy PnP that would then be reused as
        # the "best detection" in State 11 -- before the head tilt
        # has even completed. We therefore IGNORE every detection
        # until the state machine explicitly enables us at State 10.
        self.enabled = False

        # Per-cube subscription. We give each callback the cube_id by
        # closing over it in a small lambda so a single _on_detection
        # handles both feeds.
        for cube_id, topic in CUBE_ARUCO_TOPICS.items():
            node.create_subscription(
                TransformStamped,
                topic,
                self._make_cb(cube_id),
                10,
                callback_group=cb_group_io,
            )

    def _make_cb(self, cube_id):
        def _cb(msg):
            self._on_detection(msg, cube_id)
        return _cb

    # ------------------------------------------------------------------
    def enable(self):
        # Called by the state machine once Task 2 is done and the robot
        # is parked at the PICK approach pose. From this moment on,
        # detections are accepted and merged into cube_marker_in_map.
        self.enabled = True

    def _on_detection(self, msg: TransformStamped, cube_id: int):
        # Master gate (see __init__): drop everything that arrives while
        # the state machine is still running Task 2.
        if not self.enabled:
            return
        # Same convergence gating used for wall markers: composing into
        # map before AMCL is locked produces garbage.
        if not self.amcl.converged:
            return

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

        if not self.cube_seen_in_camera[cube_id]:
            self.cube_seen_in_camera[cube_id] = True
            self.node.get_logger().info(
                f"[diag] cube marker ID {cube_id} IS being detected by camera"
            )

        # Distance gate: aruco PnP accuracy degrades fast with range.
        new_distance = math.sqrt(
            aruco_in_cam.p.x() ** 2
            + aruco_in_cam.p.y() ** 2
            + aruco_in_cam.p.z() ** 2
        )
        if new_distance > MAX_DETECTION_DISTANCE:
            return

        # Closest-observation policy.
        prev_distance = self.cube_detection_distance[cube_id]
        already_have = self.cube_marker_in_map[cube_id] is not None
        if already_have and new_distance >= prev_distance:
            return

        # Compose into map AT THE TIME OF THE DETECTION (avoid drift).
        try:
            tf_map_cam = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            try:
                tf_map_cam = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.camera_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
            except Exception:
                return

        frame_map_cam = transform_to_kdl_frame(tf_map_cam)
        marker_in_map = frame_map_cam * aruco_in_cam
        self.cube_marker_in_map[cube_id] = marker_in_map
        self.cube_detection_distance[cube_id] = new_distance

    # ------------------------------------------------------------------
    def reset_cube(self, cube_id: int):
        # Called after we've placed a cube, so the next attempt does not
        # reuse a stale pose (the cube has been physically moved).
        self.cube_marker_in_map[cube_id] = None
        self.cube_detection_distance[cube_id] = float("inf")
        self.cube_seen_in_camera[cube_id] = False
