# Autonomous mobile robotics exam - Group 30

# Task 3 - Geometry and Motion Helpers 

# This file contains the math operations and manual driving functions needed for Task3. 
# It uses: 
    # nav_vel to bypass Nav2's safety limits to drive right up to the tables
    # PyKDL and TF to compute exactly the pose for the arm in the 3D space to pick up and drop 

import math
import time

import rclpy.time
from rclpy.duration import Duration

from PyKDL import Frame, Rotation, Vector

from tiago_project_group30.task2_kdl_helpers import (
    frame_to_pos_quat,
    quat_to_yaw,
    transform_to_kdl_frame,
)
from tiago_project_group30.constants import (
    CAMERA_FRAME,
    CUBE_TOP_TO_CENTER,
    DRIVE_SPEED,
    GRASP_Z_ABOVE_TOP,
    PLACE_FORWARD_OFFSET,
    PLACE_LATERAL_OFFSET,
    PLACE_TARGET_Z,
    POST_GRASP_LIFT,
    PRE_GRASP_LIFT,
    SAFE_NAV_DISTANCE,
)


class Task3Geometry:
    # ------------------------
    # Nav goal + push helpers
    # ------------------------
    def _send_cube_approach_nav(self, marker_in_map):
        # Computes exactly where the robot base should park in front of the table.
        # It sets a destination SAFE_NAV_DISTANCE away from the cube. 

        cube_x = marker_in_map.p.x()
        cube_y = marker_in_map.p.y()

        # Get the angle of the table so the robot faces it square.
        # More reliable than look to the cube. 
        pick_approach = self.aruco.pick_approach_pose
        table_yaw = quat_to_yaw(
            pick_approach.pose.orientation.x,
            pick_approach.pose.orientation.y,
            pick_approach.pose.orientation.z,
            pick_approach.pose.orientation.w,
        )

        yaw = table_yaw
        ax = cube_x - SAFE_NAV_DISTANCE * math.cos(yaw)
        ay = cube_y - SAFE_NAV_DISTANCE * math.sin(yaw)

        # Debugging: how far is the robot from the cube. 
        try:
            rx, ry, _ = self._robot_xy_yaw()
            d = math.hypot(cube_x - rx, cube_y - ry)
        except Exception:
            d = float("nan")

        self.node.get_logger().info(
            f"Cube is {d:.2f} m away (too far for arm); sending Nav2 "
            f"to cube approach pose ({ax:.2f}, {ay:.2f}) "
            f"yaw={yaw:.2f} (wall-normal)"
        )

        # Reset nav latches before sending the new goal.
        self.nav.reset_latches()
        self.nav.send_goal(ax, ay, yaw)
        self._send_time = time.time()

    def _cmd_vel_push(self, target_x, target_y, stop_dist, timeout,
                      steer, tag):
        # "Manually" drive the robot forward, bypassing Nav2 system. 
        # It checks the robot's position and stops exactly ad stop_dist from the target, or after timeout seconds.
        # It's like a proportional controller with constant speed and no integral or derivative terms, plus a timeout.

        YAW_GAIN = 1.5
        YAW_MAX = 0.5
        try:
            rx, ry, robot_yaw = self._robot_xy_yaw()
        except Exception as e:
            self.node.get_logger().warn(
                f"{tag}: TF lookup failed ({e}); stopping",
                throttle_duration_sec=2.0,
            )
            self.nav.stop()
            return None

        # check distance to target; if close enough, stop and report "done".
        dist = math.hypot(target_x - rx, target_y - ry)

        if dist <= stop_dist:
            self.nav.stop()
            # Final heading error to the target (how frontal we ended up).
            final_bearing = math.atan2(target_y - ry, target_x - rx)
            final_err = math.atan2(math.sin(final_bearing - robot_yaw),
                                   math.cos(final_bearing - robot_yaw))
            self.node.get_logger().info(
                f"{tag} done: {dist:.2f} m from target "
                f"(stop {stop_dist:.2f} m), final yaw_err={final_err:.2f} rad"
            )
            return "done"

        # Stops if timeout exceeded.
        if (time.time() - self._send_time) > timeout:
            self.nav.stop()
            self.node.get_logger().warn(
                f"{tag} timed out at {dist:.2f} m"
            )
            return "timeout"

        # Computes a turning adjustment to keep the robot aligned with the target
        angular = 0.0
        yaw_err = 0.0
        if steer:
            bearing = math.atan2(target_y - ry, target_x - rx)
            yaw_err = math.atan2(math.sin(bearing - robot_yaw),
                                 math.cos(bearing - robot_yaw))
            angular = max(-YAW_MAX, min(YAW_MAX, YAW_GAIN * yaw_err))

        # Send raw speed commands
        self.nav.publish_forward(DRIVE_SPEED, angular=angular)
        self.node.get_logger().info(
            f"{tag}: {dist:.2f} m -> stop {stop_dist:.2f} m, "
            f"yaw_err={yaw_err:.2f} rad",
            throttle_duration_sec=1.0,
        )
        return None

    def _push_reference_xy(self):
        # Determine which point on the map we're trying to drive towards. 
            # If place: use place table marker 
            # If pick: use specific cube marker. 

        if self._push_mode == "place":
            marker = self.aruco.place_marker_in_map
            if marker is None:
                return None
            return marker.p.x(), marker.p.y()
        marker = self.cube_tracker.cube_marker_in_map[self.current_cube_id]
        if marker is None:
            return None
        return marker.p.x(), marker.p.y()

    def _log_gripper_z(self, tag):
        # Debugging:   
            # Prints the height of the gripper.
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "gripper_grasping_frame",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return
        gz = tf.transform.translation.z
        self.node.get_logger().info(
            f"[diag {tag}] gripper_grasping_frame z={gz:.3f} m "
            f"(fingertips ~{gz - 0.10:.3f} m); table top ~0.30 m",
            throttle_duration_sec=0.5,
        )

    # -------------------------
    # Pose precomputation 
    # -------------------------

    def _precompute_grasp_poses(self, marker_in_map):
        # Calculates the pose for the arm to grab the cube. 
        # Converts the cube position from the map frame to the base_link frame, 
        # then applies an offset to get the final grasping pose.

        # --- camera frame marker tf to base_link ---
        marker_in_base = None
        marker_in_cam = self.cube_tracker.cube_marker_in_camera.get(
            self.current_cube_id
        )

        # Try to use the direct camera first (highly accurate)
        if marker_in_cam is not None:
            try:
                tf_base_cam = self.tf_buffer.lookup_transform(
                    "base_link", CAMERA_FRAME,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
                marker_in_base = transform_to_kdl_frame(tf_base_cam) * marker_in_cam
                frame_label = "camera"
            except Exception:
                marker_in_base = None

        # Fallback: if camera data fails, use the saved map coordinates. 
        if marker_in_base is None:
            try:
                tf_base_map = self.tf_buffer.lookup_transform(
                    "base_link", "map",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
                marker_in_base = transform_to_kdl_frame(tf_base_map) * marker_in_map
                frame_label = "map_fallback"
            except Exception as e:
                self.node.get_logger().error(f"TF base_link<-map fallita: {e}")
                return

        # --- position --- 
        cube_x = marker_in_base.p.x()
        cube_y = marker_in_base.p.y()
        # The camera sees the top of the cube, but we want to grasp the center.
        cube_center_z = marker_in_base.p.z() - CUBE_TOP_TO_CENTER
        cube_top_z = marker_in_base.p.z() 

        # --- orientation ---
        # camera angles are messy. 
        # Force the gripper to point perfectly straight down towards the floor 

        mqx, mqy, mqz, mqw = marker_in_base.M.GetQuaternion()
        cube_yaw = quat_to_yaw(mqx, mqy, mqz, mqw)
        grasp_orient = Rotation.Identity()
        grasp_orient.DoRotZ(cube_yaw) 
        grasp_orient.DoRotY(math.pi / 2.0)

        # Create final target points 
        grasp_in_base = Frame( # final grasp on the cube. 
            grasp_orient,
            Vector(cube_x, cube_y, cube_top_z + GRASP_Z_ABOVE_TOP),
        )
        pre_grasp_in_base = Frame( # pre-grasp above the cube.
            grasp_orient,
            Vector(cube_x, cube_y, cube_center_z + PRE_GRASP_LIFT),
        )

        self._grasp_pose_pos, self._grasp_pose_quat = frame_to_pos_quat(
            grasp_in_base
        )
        self._pre_grasp_pose_pos, self._pre_grasp_pose_quat = frame_to_pos_quat(
            pre_grasp_in_base
        )

        self.node.get_logger().info(
            f"[grasp] SOURCE={frame_label} cube_yaw={cube_yaw:.2f} rad "
            f"cube_base=({cube_x:.2f}, {cube_y:.2f}, {cube_center_z:.2f}) "
            f"pre_grasp=({self._pre_grasp_pose_pos[0]:.2f}, "
            f"{self._pre_grasp_pose_pos[1]:.2f}, "
            f"{self._pre_grasp_pose_pos[2]:.2f})"
        )

    def _precompute_drop_poses(self):
        # Computes cube drop poses on the place table. 
        # Computed relative to the robot's current position. 
        # So the arm can actually reach the spot without failing.
        rx, ry, robot_yaw_map = self._robot_xy_yaw()

        # Place the cube a fixed distance straight ahead of the robot. 
        drop_x_base = PLACE_FORWARD_OFFSET

        # Lateral offset (left first cube, right second cube) 
        # To ensure the second cube is not placed on the top of the first one,
        # causing maybe a collision. 
        mk = self.aruco.place_marker_in_map
        table_y_base = (-(mk.p.x() - rx) * math.sin(robot_yaw_map)
                        + (mk.p.y() - ry) * math.cos(robot_yaw_map)) if mk else 0.0
        drop_y_base = table_y_base + (PLACE_LATERAL_OFFSET if self._current_cube_idx == 0 else -PLACE_LATERAL_OFFSET)

        # Convert the table's absolute map height
        # into a relative height from the robot base, so the arm can reach it.
        tf_base_map = self.tf_buffer.lookup_transform(
            "base_link", "map",
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        frame_base_map = transform_to_kdl_frame(tf_base_map)
        drop_in_map_for_z = frame_base_map * Frame(
            Rotation.Identity(), Vector(rx, ry, PLACE_TARGET_Z)
        )
        drop_z_base = drop_in_map_for_z.p.z()
        pre_drop_z_base = drop_z_base + POST_GRASP_LIFT

        # Force the gripper to point perfectly straight down. 
        drop_in_base = Frame(Rotation.Identity(), Vector(drop_x_base, drop_y_base, drop_z_base))
        drop_in_base.M.DoRotY(math.pi / 2.0)

        pre_drop_in_base = Frame(Rotation.Identity(), Vector(drop_x_base, drop_y_base, pre_drop_z_base))
        pre_drop_in_base.M.DoRotY(math.pi / 2.0)

        self._drop_pose_pos, self._drop_pose_quat = frame_to_pos_quat(
            drop_in_base
        )
        self._pre_drop_pose_pos, self._pre_drop_pose_quat = (
            frame_to_pos_quat(pre_drop_in_base)
        )
        self.node.get_logger().info(
            f"[diag] place_z_map={PLACE_TARGET_Z:.2f} "
            f"drop_base=({self._drop_pose_pos[0]:.2f}, "
            f"{self._drop_pose_pos[1]:.2f}, {self._drop_pose_pos[2]:.2f})"
        )

    # Helper to convert global map pose into robot relative pose. 
    def _transform_to_base_link(self, frame_in_map: Frame) -> Frame:
        
        tf_base_map = self.tf_buffer.lookup_transform(
            "base_link", "map",
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        frame_base_map = transform_to_kdl_frame(tf_base_map)
        return frame_base_map * frame_in_map
