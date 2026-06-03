# Autonomous Mobile Robotics Exam - Group 30

# Task2 - Aruco Tracker

# Workflow:
    # Subscribe to the aruco_single TF topic for both PICK and PLACE markers.
    # When a new marker pose is received, compose it with the map -camera TF
    # Computes the approach pose with an offset along the marker direction
    # Publish the approach pose as a TF
    
# Notes: 
    # Waits until the robot is localized (AMCL converged) 
    # Memory the closest (and hopefully most accurate) detection for each marker. 
    # Freezes the the approach pose when the robot starts its final approach (to prevent nav2 goal jump around)

import math

from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, TransformStamped
from PyKDL import Frame, Rotation, Vector

from tiago_project_group30.constants import (
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

        # While True the system updates the approach poses with the newest. 
            # Set to False when the robot starts its final approach to prevent nav2 goal jump around.
        self._refresh_approach_poses = True

        # Stores the 3d position of the marker when it last saw it. 
        self.pick_marker_in_map = None     # PyKDL.Frame or None
        self.place_marker_in_map = None

        # Tracks how far the camera was from the marker when it last saw it. 
            #Camera estimation get worse with the distance
            # Used to keep the closest. 
        self.pick_detection_distance = float("inf")
        self.place_detection_distance = float("inf")

        # Final approach poses. 
        self.pick_approach_pose = None
        self.place_approach_pose = None

        # Flags to remember it the camera has ever seen the markers.
        self.pick_seen_in_camera = False
        self.place_seen_in_camera = False

        # Subscriptions to the ArUco detector nodes.
            # Provides marker's position relative to the camera lens.
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
        # aruco_pick_approach / aruco_place_approach.
        self.approach_timer = node.create_timer(
            0.2, self.publish_approach_frames, callback_group=cb_group_io
        )

    def freeze(self):
        # Called by the state machine on State 3 - State 4 transition.
        # Stops the approach pose from being overwritten so Nav2's target
        # doesn't drift during the final approach.
        self._refresh_approach_poses = False

    # ------------------------
    # Subscription callbacks
    # ------------------------
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
        # Do not process anything if the robot doesn't know where it is on the map yet.
        # Calculating positions now would result in completely wrong map coordinates.
        if not self.amcl.converged:
            return

        # Convert the raw camera message into a KDL Frame. 
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

        # Log the first time we see the marker in the camera.
        seen_flag = "pick_seen_in_camera" if is_pick else "place_seen_in_camera"
        if not getattr(self, seen_flag):
            setattr(self, seen_flag, True)
            self.node.get_logger().info(
                f"[diag] aruco_single {label} (ID {marker_id}) IS seeing the marker"
            )

        # If the approach pose if frozen, ignore new data. 
        already_have = (
            self.pick_marker_in_map if is_pick else self.place_marker_in_map
        ) is not None
        if already_have and not self._refresh_approach_poses:
            return

        # Computes the direct distance between the camera and the marker. 
        new_distance = math.sqrt(
            aruco_in_cam.p.x() ** 2
            + aruco_in_cam.p.y() ** 2
            + aruco_in_cam.p.z() ** 2
        )

        # Ignore markers that are too far away to be reliably detected.
        if new_distance > MAX_DETECTION_DISTANCE:
            return

        # Keep closest: If we already found the marker, update its position only if the distance is smaller than the previous detections.
        prev_distance = (
            self.pick_detection_distance if is_pick
            else self.place_detection_distance
        )
        if already_have and new_distance >= prev_distance:
            return  # this observation is no closer than what we already have

        # CRITICAL:
            # We must look up the robot's position on the map EXACTLY when the picture was taken.
            # If we use the "current" time, any movement the robot made since the picture was taken 
            # will cause the marker's calculated map position to be wrong.
        try:
            tf_map_cam = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                msg.header.stamp, # time the picture was taken. 
                timeout=Duration(seconds=0.5),
            )
        except Exception:
            return  # If we can't get the exact time data, drop it and wait for the next one. 

        # Multiply robot's map position by marker's camera position 
            # To find marker's map position.
        frame_map_cam = transform_to_kdl_frame(tf_map_cam)
        marker_in_map = frame_map_cam * aruco_in_cam

        # Save the marker's map position and detection distance.
        if is_pick:
            self.pick_marker_in_map = marker_in_map
            self.pick_detection_distance = new_distance
        else:
            self.place_marker_in_map = marker_in_map
            self.place_detection_distance = new_distance

    # ---------------------------
    # Approach-frame broadcaster 
    # ---------------------------

    # Creates and broadcasts the spot the robot should nav to. 
    # Flatten the approach pose to the 2d floor (xy plane) so the Nav2 system 
    # can drive there properly. 
    # Keep marker's original Z height purely for the arm. 

    def publish_approach_frames(self):
        # Handle the pick marker 
        if self.pick_marker_in_map is not None:
            approach = self._compute_horizontal_approach(
                self.pick_marker_in_map, APPROACH_DISTANCE
            )
            self._broadcast_frame(approach, PICK_APPROACH_FRAME)

            # Save the pose (if it's first time or if we're still allowing updates)
            first_time = self.pick_approach_pose is None
            if first_time or self._refresh_approach_poses:
                self.pick_approach_pose = self._frame_to_pose_stamped(approach)
                if first_time:
                    self.node.get_logger().info(
                        f"PICK approach localized at map "
                        f"({approach.p.x():.2f}, {approach.p.y():.2f})"
                    )

        # Handle the place marker. Same logic.
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
        # Computes a safe position in front of the marker for the mobile base.
        # Find the direction marker (Z axis). 

        marker_z = marker_in_map.M * Vector(0.0, 0.0, 1.0)

        # Flatten the direction into 2D floor. 
        horiz = math.sqrt(marker_z.x() ** 2 + marker_z.y() ** 2)
        if horiz > 0.05:
            dx = marker_z.x() / horiz
            dy = marker_z.y() / horiz
        else:
            # Fallback if the marker is pointing straight up or down. 
            dx, dy = 1.0, 0.0

        # Create target point 
        ax = marker_in_map.p.x() + distance * dx
        ay = marker_in_map.p.y() + distance * dy
        az = marker_in_map.p.z()  # keep marker height (useful for Task 3 arm approach)

        # Computes yaw so the robot directly faces the marker. 
        yaw = math.atan2(marker_in_map.p.y() - ay, marker_in_map.p.x() - ax)

        # Flat orientation 
        return Frame(Rotation.RPY(0.0, 0.0, yaw), Vector(ax, ay, az))

    def _broadcast_frame(self, frame: Frame, child_frame_id: str):
        # Converts KDL Frame to ROS TransformStamped and broadcasts it.
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
        # Converts KDL Frame to ROS PoseStamped. 
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
