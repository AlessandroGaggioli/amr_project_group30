# Autonomous Mobile Robotics Exam - Group 30
#
# Task 3 - Cube aruco Tracker 
#
# Tracks the aruco markers placed on top of the cubes. 


import math

import rclpy.time
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
from PyKDL import Frame, Rotation, Vector

from tiago_project_group30.constants import (
    CAMERA_FRAME,
    CUBE_ARUCO_TOPICS,
    CUBE_PICK_SEQUENCE,
    MAX_DETECTION_DISTANCE,
)
from tiago_project_group30.task2_kdl_helpers import transform_to_kdl_frame


class CubeTracker:

    def __init__(self, node, tf_buffer, amcl, cb_group_io,
                 map_frame: str = "map"):
        self.node = node
        self.tf_buffer = tf_buffer
        self.amcl = amcl
        self.map_frame = map_frame
        self.camera_frame = CAMERA_FRAME

        # Dictionary to store the calculated global map coordinates for each cube. 
        self.cube_marker_in_map = {cid: None for cid in CUBE_PICK_SEQUENCE}

        # Dictionary to store the raw position of the cube relative to the camera lens. 
        self.cube_marker_in_camera = {cid: None for cid in CUBE_PICK_SEQUENCE}

        # Tracks how far the camera was from the cube to enforce the keep closest policy. 
        self.cube_detection_distance = {
            cid: float("inf") for cid in CUBE_PICK_SEQUENCE
        }

        # Flags to remember if the camera has ever seen each cube. 
        self.cube_seen_in_camera = {cid: False for cid in CUBE_PICK_SEQUENCE}

        # The camera might see a cube from across the room while driving.
        # keep this false to ignore all data unitl the robot is in front of the table.
        self.enabled = False

        # Once the robot is in front of the table
        # Lock the coordinates of the cube. Prevents noisy camera frame from shifting the target during the grasp.
        self.frozen = False

        # Subscribe to the aruco TF topics. 
        # The callback will process the detections and update the cube_marker_in_map.
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
        # Turns on the tracker. 
        self.enabled = True

    def freeze(self):
        # Locks the cube's coordinates before grasping. 
        self.frozen = True

    def unfreeze(self):
        # Unlock the tracker so it can look to the next cube. 
        self.frozen = False

    def _on_detection(self, msg: TransformStamped, cube_id: int):
        # Ignore data if the tracker is turned off 
        if not self.enabled:
            return
        # Ignore data if the target is already frozen. 
        if self.frozen:
            return
        # Ignore data if AMCL is not converged. 
        if not self.amcl.converged:
            return

        # Convert raw camera message into KDL Frame. 
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
                f"[diag] cube marker ID {cube_id} is being detected by camera"
            )

        # Calculate the direct distance between the camera and the cube.
        new_distance = math.sqrt(
            aruco_in_cam.p.x() ** 2
            + aruco_in_cam.p.y() ** 2
            + aruco_in_cam.p.z() ** 2
        )
        # Ignore cubes that are too far away
        if new_distance > MAX_DETECTION_DISTANCE:
            return

        # Closest-observation policy. Only update the cube's position if this new 
        # picture was taken from closer up than our previous best picture.
        prev_distance = self.cube_detection_distance[cube_id]
        already_have = self.cube_marker_in_map[cube_id] is not None
        if already_have and new_distance >= prev_distance:
            return

        # Calculate the exact map coordinates of the cube.
        # First, try to get the robot's position at the exact timestamp the picture was taken.
        # Ifif fails (due to CPU load probably) fallback to the latest known position.

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
                    timeout=Duration(seconds=0.2),
                )
            except Exception:
                return # If both fail, drop the image and wait for the next one

        # Multiply the robot's map position by the cube's camera position 
        # to get the absolute map coordinates. 
        frame_map_cam = transform_to_kdl_frame(tf_map_cam)
        marker_in_map = frame_map_cam * aruco_in_cam
        self.cube_marker_in_map[cube_id] = marker_in_map
        self.cube_marker_in_camera[cube_id] = aruco_in_cam
        self.cube_detection_distance[cube_id] = new_distance

    # ------------------------------------------------------------------
    def reset_cube(self, cube_id: int):
        # Clears the memory for a specific cube. Called after a cube is picked 
        # and moved, so the robot doesn't accidentally try to use its old coordinates later.
        self.cube_marker_in_map[cube_id] = None
        self.cube_marker_in_camera[cube_id] = None
        self.cube_detection_distance[cube_id] = float("inf")
        self.cube_seen_in_camera[cube_id] = False
