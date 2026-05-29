# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# ArUco tracker (Lab 3 pattern, adapted for 2-D base navigation):
#
#   - Each aruco_single node runs with reference_frame=''
#     -> publishes the marker pose in the camera optical frame.
#   - This component subscribes to /aruco_pick/aruco_single/transform
#     and /aruco_place/aruco_single/transform and composes the marker
#     pose into the MAP frame AT THE TIME OF THE DETECTION
#     (msg.header.stamp). This freezes the marker at its true world
#     position; recomposing later with a fresh map<-camera TF would
#     make the result slide as the camera moves.
#   - A 5 Hz timer derives the approach frame (offset along marker +Z,
#     projected onto the horizontal plane) and broadcasts it as
#     `aruco_pick_approach` / `aruco_place_approach`. The first
#     successful composition latches a PoseStamped that Nav2 navigates to.
#
# Composition is gated on AMCL convergence: until AMCL is locked, the
# map<-camera TF is built on top of AMCL's prior (uniform or wrong) and
# would produce a garbage map-frame position.

import math

from rclpy.duration import Duration
from rclpy.qos import QoSProfile

import rclpy.time

from geometry_msgs.msg import PoseStamped, TransformStamped
from PyKDL import Frame, Rotation, Vector

from tiago_project_group30.task2_constants import (
    APPROACH_DISTANCE,
    CAMERA_FRAME,
    MAX_DETECTION_DISTANCE,
    PICK_APPROACH_FRAME,
    PLACE_APPROACH_FRAME,
)
from tiago_project_group30.task2_kdl_helpers import transform_to_kdl_frame


class ArucoTracker:
    def __init__(self, node, tf_buffer, tf_broadcaster, amcl, cb_group_io,
                 map_frame: str = "map"):
        self.node = node
        self.tf_buffer = tf_buffer
        self.tf_broadcaster = tf_broadcaster
        self.amcl = amcl
        self.map_frame = map_frame
        self.camera_frame = CAMERA_FRAME

        # While True, approach poses are continuously *replaced* with the
        # latest aruco observation (so noisy long-range first detections
        # are overwritten as the robot gets closer). Set to False once we
        # transition to State 4 so the nav goal does not chase a moving
        # estimate during the final approach.
        self._refresh_approach_poses = True

        # Marker poses in MAP frame, composed at observation time.
        # We do NOT store the raw camera-frame pose and recompose later,
        # because doing so multiplies a *current* map<-camera TF by an
        # *old* marker_in_camera (taken when the robot was elsewhere) and
        # produces a frame that slides around the map as the robot moves.
        self.pick_marker_in_map = None     # PyKDL.Frame or None
        self.place_marker_in_map = None

        # Distance from the camera to the marker at the moment of the
        # observation that produced *_marker_in_map. ArUco pose accuracy
        # degrades fast with distance (a 25 cm marker at 5 m is only ~30
        # px wide, so its estimated depth/orientation can be off by a
        # meter+). We use these to KEEP THE CLOSEST OBSERVATION.
        self.pick_detection_distance = float("inf")
        self.place_detection_distance = float("inf")

        # Latched approach poses in map frame (PoseStamped)
        self.pick_approach_pose = None
        self.place_approach_pose = None

        # Diagnostics: True once the aruco_single node has ever published
        # a transform for that marker (regardless of TF lookup success).
        self.pick_seen_in_camera = False
        self.place_seen_in_camera = False

        # Subscriptions: aruco_single runs with reference_frame='' ->
        # publishes marker pose in the CAMERA optical frame. The topic is
        # private to the node so with namespace='aruco_pick'/name=
        # 'aruco_single' it resolves to:
        #     /aruco_pick/aruco_single/transform
        # NOT /aruco_pick/transform (that one only appears in `ros2 topic
        # list` because subscribing creates an entry, even with no
        # publisher).
        node.create_subscription(
            TransformStamped, "/aruco_pick/aruco_single/transform",
            self._aruco_pick_cb, 10, callback_group=cb_group_io,
        )
        node.create_subscription(
            TransformStamped, "/aruco_place/aruco_single/transform",
            self._aruco_place_cb, 10, callback_group=cb_group_io,
        )

        # Approach-frame publisher timer. At 5 Hz, if we have a stored
        # marker_in_map, compute the approach offset and broadcast it as
        # aruco_pick_approach / aruco_place_approach. The first successful
        # broadcast latches the corresponding approach pose for Nav2.
        self.approach_timer = node.create_timer(
            0.2, self.publish_approach_frames, callback_group=cb_group_io
        )

    def freeze(self):
        # Called by the state machine on State 3 -> State 4 transition.
        # Stops the approach pose from being overwritten so Nav2's target
        # doesn't drift during the final approach.
        self._refresh_approach_poses = False

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
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
        # Gate ArUco -> map composition on AMCL convergence. Until AMCL
        # has converged, the TF map<-camera is built on top of AMCL's
        # prior (uniform / uninitialized), so composing the marker pose
        # with it gives a garbage map-frame position (we observed a PICK
        # approach landing inside a wall when an ArUco detection was
        # processed 1.3 s after /reinitialize_global_localization).
        if not self.amcl.converged:
            return

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
            self.node.get_logger().info(
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

        # ArUco pose accuracy degrades with distance. The camera-frame
        # translation of the marker IS the distance from camera to marker,
        # so we compute it directly without any TF.
        new_distance = math.sqrt(
            aruco_in_cam.p.x() ** 2
            + aruco_in_cam.p.y() ** 2
            + aruco_in_cam.p.z() ** 2
        )

        # Reject detections beyond MAX_DETECTION_DISTANCE (PnP too noisy).
        if new_distance > MAX_DETECTION_DISTANCE:
            return

        # KEEP-CLOSEST policy: only overwrite the stored marker_in_map if
        # this new observation is closer than the previous one.
        prev_distance = (
            self.pick_detection_distance if is_pick
            else self.place_detection_distance
        )
        if already_have and new_distance >= prev_distance:
            return  # this observation is no closer than what we already have

        # CRITICAL: compose into MAP frame NOW, using the camera TF AT
        # THE TIME OF THE DETECTION (msg.header.stamp). This freezes the
        # marker at its true world position.
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
    # Lab 3 computes the approach by offsetting APPROACH_DISTANCE along
    # the marker's Z axis and then applying DoRotY(pi/2) so the frame's X
    # axis points toward the marker. That works well for the arm approach
    # (Task 3) but for 2-D base navigation the resulting frame has
    # non-zero pitch/roll (because the marker's Y is vertical/down), which
    # looks wrong in RViz and produces an awkward Nav2 goal orientation.
    #
    # Instead we project the marker's Z axis onto the horizontal XY plane
    # to get the approach direction, then build the approach frame with a
    # flat orientation (roll=0, pitch=0, yaw toward marker). The position
    # keeps the marker's Z height so Task 3 can reuse the same frame.
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
                    self.node.get_logger().info(
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
                    self.node.get_logger().info(
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
        t.header.stamp = self.node.get_clock().now().to_msg()
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
        ps.header.stamp = self.node.get_clock().now().to_msg()
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
