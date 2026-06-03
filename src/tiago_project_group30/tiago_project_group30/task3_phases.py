# Autonomous Mobile Robotics Exam - Group 30

# Task 3 - Phase handlers. 

# Individual logic blocks of the state machine. 

import math
import time

from tiago_project_group30.constants import (
    CUBE_APPROACH_DISTANCE,
    CUBE_PICK_SEQUENCE,
    DRIVE_SPEED,
    HEAD_SCAN_PANS,
    HEAD_TILT_DOWN_FOR_CUBE,
    PLACE_APPROACH_DISTANCE,
    SAFE_NAV_DISTANCE,
)
from tiago_project_group30.common_states import Phase


class Task3Phases:
    # -------------
    # Task 2 Nav
    # -------------
    def state_4_pick(self, approach_nav_timeout):
        # CHANGE vs. Task 2: on success advance to HEAD_TILT (cube pick flow).
        # Once the robot arrives at pick table, do not stop. 
        return self._nav_to_approach(
            self.aruco.pick_approach_pose,
            next_state=Phase.HEAD_TILT,
            timeout=approach_nav_timeout,
            label="PICK",
            success_log=(
                f"PICK approach reached, starting Task 3 pick of "
                f"cube ID {self.current_cube_id}"
            ),
            send_log=(
                f"State 4: nav to PICK approach "
                f"(cube #{self._current_cube_idx + 1} of {len(CUBE_PICK_SEQUENCE)})"
            ),
        )

    def state_5_place(self, approach_nav_timeout):
        # CHANGE vs. Task 2: on success advance to REFRESH_PLACE. 
        # Once the robot arrives at place table, do not stop.

        def _on_success():
            self._send_time = time.time()
            self.state = Phase.REFRESH_PLACE
        return self._nav_to_approach(
            self.aruco.place_approach_pose,
            next_state=Phase.REFRESH_PLACE,
            timeout=approach_nav_timeout,
            label="PLACE",
            success_log="PLACE approach reached, re-detecting marker up close",
            send_log=(
                f"State 5: nav to PLACE approach "
                f"(cube #{self._current_cube_idx + 1})"
            ),
            on_success=_on_success,
        )

    # -----------------
    # Task 3 Object Approach Phases
    # -----------------
    def _phase_head_tilt(self):

        # Turn on the camera tracker for the cubes now (after arrived in front of the table). 
        # Prevents the robot from getting  confused. 
        self.cube_tracker.enable()
   
        # Tilt the head straight down to look at the table surface.
        self.node.get_logger().info(
            f"HEAD_TILT: tilt={HEAD_TILT_DOWN_FOR_CUBE} rad, pan=0.0 rad"
        )
        self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, 0.0)
        self._send_time = time.time()
        self._head_scan_active = False
        self._head_scan_idx = 0
        self._head_scan_time = None

        # Move to the next phase. 
        self.state = Phase.WAIT_CUBE

    def _phase_wait_cube(self, detection_timeout):
        # Checks camera feed to find  the target cube. 
        cube_id = self.current_cube_id
        marker = self.cube_tracker.cube_marker_in_map[cube_id]

        # If the camera sees the cube. 
        if marker is not None:
            self.node.get_logger().info(
                f"Cube ID {cube_id} marker localized at map "
                f"({marker.p.x():.2f}, {marker.p.y():.2f}, {marker.p.z():.2f})"
            )

            # First detection: see the cube, but far away to grab it. 
            # Computes a new navigation path to get closer. 
            if not self._cube_approached:
                try:
                    self._send_cube_approach_nav(marker)
                except Exception as e:
                    self.node.get_logger().error(
                        f"Failed to compute cube approach nav goal: {e}"
                    )
                    time.sleep(0.5)
                    return
                self.state = Phase.WAIT_CUBE_NAV
                return
            
            # Second try: See the cube, robot is in front of it.
            # Lock the coordinates. Computes arm movements. Start grasping. 
            self.cube_tracker.freeze()
            try:
                self._precompute_grasp_poses(marker)
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to compute grasp poses: {e}"
                )
                time.sleep(0.5)
                return
            self._start_grasp_sequence()
            return
        
        # No marker yet: start a head scan only when the cube is not in the
        # FOV. This keeps the head still for a brief window so any immediate
        # detection isn't invalidated by motion.
        scan_after = min(2.0, float(detection_timeout))
        if not self._head_scan_active:
            if (time.time() - self._send_time) < scan_after:
                self.node.get_logger().info(
                    f"Waiting for cube ID {cube_id} to enter FOV...",
                    throttle_duration_sec=2.0,
                )
                return
            self._head_scan_active = True
            self._head_scan_idx = 0
            self._head_scan_time = time.time()
            pan = HEAD_SCAN_PANS[self._head_scan_idx]
            self.node.get_logger().info(
                f"Cube not visible, starting head scan pan={pan:.2f} rad"
            )
            self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, pan)
            return

        self._advance_head_scan("Head scan")

    def _phase_wait_cube_nav(self, approach_nav_timeout):
        # Currently driving closer to the specific cube. Wait for Nav2. 
        res = self._consume_nav_result()

        if res is True:
            self.node.get_logger().info(
                "Cube approach Nav2 reached (within inflation limit); "
                "closing remaining gap with a /cmd_vel push"
            )

            # Recenter the head to look straight down at the cube 
            # and start the "manual" driving to get in range. 
            self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, 0.0)
            time.sleep(1.0)
            self._push_mode = "pick"
            self._send_time = time.time()
            self.state = Phase.PUSH
            return
        
        if res is False:
            self.node.get_logger().warn(
                "Cube approach nav did not succeed; attempting grasp anyway"
            )
            self._cube_approached = True
            self.state = Phase.WAIT_CUBE
            return
        
        self._nav_timeout_guard(
            approach_nav_timeout, "Cube approach nav timed out, cancelling"
        )

    # -------------------------
    # Manual driving (push and back_out)
    #--------------------------


    def _phase_push(self):
        # Drives forward until we're to the right distance from the target to use the arm. 
        is_pick = self._push_mode != "place"
        ref = self._push_reference_xy()

        if ref is None:
            self.node.get_logger().warn(
                f"push ({self._push_mode}): reference marker missing, "
                "proceeding anyway"
            )
            self.nav.stop()
            if is_pick:
                self._cube_approached = True
                self.state = Phase.WAIT_CUBE
            else:
                self._start_drop_sequence()
            return

        # Choose to stop distance based on if we're picking or placing.
        stop_dist = CUBE_APPROACH_DISTANCE if is_pick else PLACE_APPROACH_DISTANCE
        
        result = self._cmd_vel_push(
            ref[0], ref[1], stop_dist=stop_dist,
            timeout=40.0, steer=True, tag=f"push ({self._push_mode})",
        )

        # Reached the target distance. 
        if result is not None:
            if is_pick:
                self._cube_approached = True
                if result == "done" or result == "timeout":
                    # Tilt the head. 
                    self.arm.tilt_head(-1.15, 0.0)
                    # Pause to let the base inertia settle.
                    time.sleep(1.5)
                    self.cube_tracker.reset_cube(self.current_cube_id)
                    self.cube_tracker.unfreeze()

                self.state = Phase.WAIT_CUBE
            else:
                # Place table, start the drop sequence.
                self._start_drop_sequence()

    def _phase_back_out(self):
        # Opposite of push. 
        BACK_TIMEOUT = 60.0

        next_state = Phase.NAV_PLACE if self._push_mode != "place" else Phase.NEXT_OR_DONE

        ref = self._push_reference_xy()
        if ref is None:
            self.node.get_logger().warn(
                f"back_out ({self._push_mode}): reference marker missing, "
                "proceeding anyway"
            )
            self.nav.stop()
            self.state = next_state
            return

        try:
            rx, ry, _ = self._robot_xy_yaw()
        except Exception as e:
            self.node.get_logger().warn(
                f"back_out ({self._push_mode}): TF lookup failed ({e}); stopping",
                throttle_duration_sec=2.0,
            )
            self.nav.stop()
            return

        dist = math.hypot(ref[0] - rx, ref[1] - ry)

        # Far enough from the surface. stop and go to the next leg.
        if dist >= SAFE_NAV_DISTANCE:
            self.nav.stop()
            self.node.get_logger().info(
                f"back_out ({self._push_mode}) done: {dist:.2f} m from ref "
                f"(target {SAFE_NAV_DISTANCE:.2f} m), clear of inflation"
            )
            # Clean nav latches before handing back to a Nav2 leg.
            self.nav.reset_latches()
            self.state = next_state
            return

        if (time.time() - self._send_time) > BACK_TIMEOUT:
            self.nav.stop()
            self.node.get_logger().warn(
                f"back_out ({self._push_mode}) timed out at {dist:.2f} m; "
                "proceeding anyway"
            )
            self.state = next_state
            return

        # Still inside the zone . keep reversing (negative speed).
        self.nav.publish_forward(-DRIVE_SPEED)
        self.node.get_logger().info(
            f"back_out ({self._push_mode}): {dist:.2f} m -> target "
            f"{SAFE_NAV_DISTANCE:.2f} m",
            throttle_duration_sec=1.0,
        )

    def _phase_refresh_place(self, timeout):
        # Arrived at place table. Take a fresh picture to aim the drop better. 
        if not self._arm_goal_sent_for_state(Phase.REFRESH_PLACE):
            self.node.get_logger().info(
                "REFRESH_PLACE: re-detecting place marker up close before drop"
            )
            
            # Allow the system to overwrite the old map coordinates. 
            self.aruco.place_detection_distance = float("inf")
            self.aruco._refresh_approach_poses = True
            
            # Start scanning the table to find the marker again. 
            self._head_scan_idx = 0
            self._head_scan_time = time.time()
            self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, HEAD_SCAN_PANS[0])
            self._mark_arm_goal_sent(Phase.REFRESH_PLACE)
            self._send_time = time.time()
            return

        # If the camera sees the marker up close 
        refreshed = self.aruco.place_detection_distance < 1.5
        timed_out = (time.time() - self._send_time) > timeout

        if refreshed or timed_out:
            if refreshed:
                self.node.get_logger().info(
                    f"Place marker refreshed at "
                    f"{self.aruco.place_detection_distance:.2f} m; pushing"
                )
            else:
                self.node.get_logger().warn(
                    "Place marker not re-detected up close; "
                    "proceeding with frozen value"
                )

            self.aruco._refresh_approach_poses = False  # re-freeze
            self._push_mode = "place"
            self._clear_arm_goal_sent()
            self._send_time = time.time()

            # move to push phase. 
            self.state = Phase.PUSH
            return

        # Not refreshed yet. keep sweeping the head. 
        self._advance_head_scan("Place marker scan")

    def _phase_next_or_done(self):
        # Cube successfully placed. 
        finished_cube_id = self.current_cube_id
        self.node.get_logger().info(
            f"NEXT_OR_DONE: cube ID {finished_cube_id} placed"
        )

        # Advance to next cube. 
        self._current_cube_idx += 1

        if self._current_cube_idx >= len(CUBE_PICK_SEQUENCE):
            self.node.get_logger().info(
                "All cubes placed -> Task 3 done"
            )
            self.state = Phase.DONE
            return
        
        # Prepare the system to grab the next cube. 
        next_id = self.current_cube_id
        self.node.get_logger().info(
            f"Next cube to pick: ID {next_id} -> nav back to PICK"
        )

        # Delete any old camera data. 
        self.cube_tracker.reset_cube(next_id)
        self.cube_tracker.unfreeze()
        self._cube_approached = False
        self.nav.reset_latches()

        # loop back to pick up. 
        self.state = Phase.NAV_PICK
