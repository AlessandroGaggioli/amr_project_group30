# TASK 3 of Autonomous Mobile Robotics Exam - Group 30
#
# Self-contained state machine that performs Task 2 (random search +
# AMCL alignment + navigation to PICK / PLACE) AND the Task 3 pick-and-
# place sub-flow for two cubes (ID 63 first, ID 582 second).
#
# Modelled on Lab 4 'state_machine_not_blocking' (same polling /
# one-shot-flag pattern). States 0-6 are functionally identical to the
# Task 2 StateMachine; the only changes are:
#   * State 4 (PICK reached) -> jumps to State 10 (Task 3 cube pick)
#       instead of State 5.
#   * State 5 (PLACE reached) -> jumps to State 20 (Task 3 cube drop)
#       instead of State 6.
# The new Task 3 sub-states are documented in their tick methods below.

import math
import time
from threading import Event

import rclpy
import rclpy.time
from rclpy.duration import Duration

from PyKDL import Frame, Rotation, Vector

from tiago_project_group30.task2_kdl_helpers import transform_to_kdl_frame
from tiago_project_group30.task3_constants import (
    CUBE_APPROACH_DISTANCE,
    CUBE_PICK_SEQUENCE,
    CUBE_TOP_TO_CENTER,
    DRIVE_SPEED,
    HEAD_SCAN_DWELL,
    HEAD_SCAN_PANS,
    HEAD_TILT_DOWN_FOR_CUBE,
    PLACE_FORWARD_OFFSET,
    PLACE_TARGET_Z,
    POST_GRASP_LIFT,
    PRE_GRASP_LIFT,
)


class Task3StateMachine:
    def __init__(self, node, arm, nav, amcl, aruco, sampler,
                 gripper, link_attacher, cube_tracker, tf_buffer):
        self.node = node
        self.arm = arm
        self.nav = nav
        self.amcl = amcl
        self.aruco = aruco
        self.sampler = sampler
        # Task 3-only components.
        self.gripper = gripper
        self.link_attacher = link_attacher
        self.cube_tracker = cube_tracker
        self.tf_buffer = tf_buffer

        self.state = 0
        self.finished = False
        self.search_phase = "sample"
        self._send_time = None

        # Task 3 progress tracking.
        self._current_cube_idx = 0   # index into CUBE_PICK_SEQUENCE
        # State 11 head-scan progress (index into HEAD_SCAN_PANS).
        self._scan_idx = 0
        # One-shot latch: True after we have navigated to the cube
        # approach pose (so re-entering State 11 with a fresh marker
        # goes straight to grasping instead of looping nav forever).
        self._cube_approached = False
        # Cached grasp / place poses (computed once per cube to avoid
        # recomputing inside every tick at 20 Hz).
        self._grasp_pose_pos = None
        self._grasp_pose_quat = None
        self._pre_grasp_pose_pos = None
        self._pre_grasp_pose_quat = None
        self._drop_pose_pos = None
        self._drop_pose_quat = None
        self._pre_drop_pose_pos = None
        self._pre_drop_pose_quat = None

    @property
    def current_cube_id(self) -> int:
        return CUBE_PICK_SEQUENCE[self._current_cube_idx]

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self, executor_ready: Event):
        self.node.get_logger().info("State machine: waiting for executor...")
        executor_ready.wait()
        self.node.get_logger().info("Executor ready, giving the stack 10s to come up")
        time.sleep(10.0)

        PLANNING_TIMEOUT = 6.0
        SEARCH_NAV_TIMEOUT = 25.0
        APPROACH_NAV_TIMEOUT = 180.0
        SEARCH_SPIN_TIMEOUT = 15.0
        # Task 3-side timeouts.
        ARM_MOTION_TIMEOUT = 20.0       # MoveIt2 plan+execute per arm goal
        GRIPPER_TIMEOUT = 5.0
        ATTACH_TIMEOUT = 3.0
        CUBE_DETECTION_TIMEOUT = 15.0   # max wait for cube marker after head tilt

        while rclpy.ok() and not self.finished:
            try:
                self.arm.update_flags()
                self.nav.update_flags()
                self.nav.update_drive_flags()
                self.amcl.update_spin_flags()
                self.amcl.update_amcl_flags()
                self.gripper.update_flags()

                if self.state == 0:
                    self._tick_state_0_arm_tuck()
                elif self.state == 1:
                    self._tick_state_1_wait_arm(PLANNING_TIMEOUT)
                elif self.state == 2:
                    self._tick_state_2_amcl_localization()
                elif self.state == 3:
                    if self._tick_state_3_random_search(
                        SEARCH_NAV_TIMEOUT, SEARCH_SPIN_TIMEOUT
                    ):
                        continue
                elif self.state == 4:
                    if self._tick_state_4_pick(APPROACH_NAV_TIMEOUT):
                        continue
                elif self.state == 5:
                    if self._tick_state_5_place(APPROACH_NAV_TIMEOUT):
                        continue
                elif self.state == 6:
                    self._tick_state_6_done()
                    break

                # ---- Task 3 pick sub-flow ----
                elif self.state == 10:
                    self._tick_state_10_head_tilt(CUBE_DETECTION_TIMEOUT)
                elif self.state == 11:
                    self._tick_state_11_wait_cube_visible(CUBE_DETECTION_TIMEOUT)
                elif self.state == 12:
                    self._tick_state_12_open_gripper_pre_grasp(GRIPPER_TIMEOUT)
                elif self.state == 13:
                    self._tick_state_13_arm_pre_grasp(ARM_MOTION_TIMEOUT)
                elif self.state == 14:
                    self._tick_state_14_arm_grasp(ARM_MOTION_TIMEOUT)
                elif self.state == 15:
                    self._tick_state_15_close_gripper(GRIPPER_TIMEOUT)
                elif self.state == 16:
                    self._tick_state_16_attach(ATTACH_TIMEOUT)
                elif self.state == 17:
                    self._tick_state_17_arm_lift_after_grasp(ARM_MOTION_TIMEOUT)
                elif self.state == 18:
                    self._tick_state_18_arm_carry(ARM_MOTION_TIMEOUT)
                elif self.state == 19:
                    self._tick_state_19_wait_cube_nav(APPROACH_NAV_TIMEOUT)

                # ---- Task 3 drop sub-flow ----
                elif self.state == 20:
                    self._tick_state_20_arm_pre_drop(ARM_MOTION_TIMEOUT)
                elif self.state == 21:
                    self._tick_state_21_arm_drop(ARM_MOTION_TIMEOUT)
                elif self.state == 22:
                    self._tick_state_22_open_gripper_release(GRIPPER_TIMEOUT)
                elif self.state == 23:
                    self._tick_state_23_detach(ATTACH_TIMEOUT)
                elif self.state == 24:
                    self._tick_state_24_arm_lift_after_drop(ARM_MOTION_TIMEOUT)
                elif self.state == 25:
                    self._tick_state_25_arm_carry_back(ARM_MOTION_TIMEOUT)
                elif self.state == 26:
                    self._tick_state_26_next_or_done()
                elif self.state == 27:
                    self._tick_state_27_wait_drive_on_heading()

            except Exception as e:
                import traceback
                self.node.get_logger().error(
                    f"Exception in state machine: {e}\n{traceback.format_exc()}"
                )

            time.sleep(0.05)

    # ==================================================================
    # States 0-3: same as Task 2.
    # ==================================================================
    def _tick_state_0_arm_tuck(self):
        self.node.get_logger().info("State 0: tuck arm to HOME (MoveIt)")
        self.arm.move_to_home(tilt_head=True)
        self._send_time = time.time()
        self.state = 1

    def _tick_state_1_wait_arm(self, planning_timeout):
        if self.arm.motion_done:
            self.node.get_logger().info(
                "Arm at HOME, starting AMCL global localization"
            )
            self.state = 2
        elif (
            not self.arm.motion_started
            and (time.time() - self._send_time) > planning_timeout
        ):
            self.node.get_logger().warn("Arm planning timed out, retrying")
            self.state = 0

    def _tick_state_2_amcl_localization(self):
        amcl = self.amcl
        if amcl.loc_substate == 0:
            self.node.get_logger().info(
                "State 2a: requesting AMCL global localization"
            )
            if amcl.request_global_localization():
                time.sleep(2.0)
                amcl.loc_substate = 1
            else:
                time.sleep(2.0)
        elif amcl.loc_substate == 1:
            self.node.get_logger().info(
                "State 2b: clearing local costmap before spin"
            )
            amcl.clear_local_costmap()
            time.sleep(0.5)
            amcl.loc_substate = 2
        elif amcl.loc_substate == 2:
            amcl.localization_spin_count += 1
            self.node.get_logger().info(
                f"State 2c: spin attempt "
                f"#{amcl.localization_spin_count} (target_yaw=2*pi)"
            )
            amcl.send_spin(2.0 * math.pi)
            self._send_time = time.time()
            amcl.loc_substate = 3
        elif amcl.loc_substate == 3:
            if amcl.converged:
                self.node.get_logger().info(
                    "State 2d: AMCL aligned -- proceeding to search"
                )
                if amcl.spin_active and amcl.spin_goal_handle is not None:
                    amcl.cancel_spin()
                self.state = 3
                return
            if amcl.spin_done:
                if amcl.localization_spin_count >= 3:
                    self.node.get_logger().warn(
                        f"AMCL not converged after "
                        f"{amcl.localization_spin_count} spins, "
                        "proceeding anyway"
                    )
                    self.state = 3
                    return
                self.node.get_logger().info(
                    "Spin done, AMCL not converged yet -- "
                    "clearing costmap and re-spinning"
                )
                amcl.spin_done = False
                amcl.loc_substate = 1

    def _tick_state_3_random_search(self, search_nav_timeout, search_spin_timeout):
        amcl = self.amcl
        nav = self.nav
        aruco = self.aruco

        if (
            aruco.pick_approach_pose is not None
            and aruco.place_approach_pose is not None
        ):
            if nav.goal_active:
                self.node.get_logger().info(
                    "Both markers seen mid-search, cancelling nav"
                )
                nav.cancel()
            if amcl.spin_active and amcl.spin_goal_handle is not None:
                self.node.get_logger().info(
                    "Both markers seen mid-spin, cancelling spin"
                )
                amcl.cancel_spin()
            nav.goal_active = False
            nav.goal_done = False
            nav.goal_succeeded = False
            amcl.spin_active = False
            amcl.spin_done = False
            amcl.spin_succeeded = False
            self.search_phase = "sample"
            aruco.freeze()
            self.node.get_logger().info(
                f"Freezing PICK approach at "
                f"({aruco.pick_approach_pose.pose.position.x:.2f}, "
                f"{aruco.pick_approach_pose.pose.position.y:.2f}) and "
                f"PLACE approach at "
                f"({aruco.place_approach_pose.pose.position.x:.2f}, "
                f"{aruco.place_approach_pose.pose.position.y:.2f})"
            )
            self.state = 4
            return True

        if self.search_phase == "nav" and nav.goal_done:
            nav_ok = nav.goal_succeeded
            nav.goal_done = False
            if nav_ok:
                self.node.get_logger().info(
                    "Waypoint reached, sending Spin 2*pi"
                )
                amcl.send_spin(2.0 * math.pi)
                self.search_phase = "spin"
                self._send_time = time.time()
            else:
                self.node.get_logger().warn(
                    "Waypoint nav failed -> sampling next"
                )
                self.search_phase = "sample"

        if self.search_phase == "spin" and amcl.spin_done:
            spin_ok = amcl.spin_succeeded
            amcl.spin_done = False
            if not spin_ok:
                self.node.get_logger().warn(
                    "Waypoint spin aborted -> sampling next"
                )
            self.search_phase = "sample"

        if self.search_phase == "nav" and nav.goal_active:
            if (
                not nav.timeout_fired
                and (time.time() - self._send_time) > search_nav_timeout
            ):
                self.node.get_logger().warn(
                    "Nav goal timed out, cancelling -> sample next"
                )
                nav.cancel()
                nav.timeout_fired = True
            return True

        if self.search_phase == "spin" and amcl.spin_active:
            if (
                not nav.timeout_fired
                and (time.time() - self._send_time) > search_spin_timeout
            ):
                self.node.get_logger().warn(
                    "Spin timed out, cancelling -> sample next"
                )
                amcl.cancel_spin()
                nav.timeout_fired = True
            return True

        if self.search_phase == "sample":
            if self.sampler.costmap_msg is None:
                self.node.get_logger().info(
                    "Waiting for /global_costmap/costmap...",
                    throttle_duration_sec=2.0,
                )
                time.sleep(0.5)
                return True
            xy = self.sampler.sample_random_xy()
            if xy is None:
                self.node.get_logger().warn(
                    "Sample returned None, clearing visited "
                    "list and retrying",
                    throttle_duration_sec=2.0,
                )
                self.sampler.visited_waypoints = []
                time.sleep(0.5)
                return True
            x, y = xy
            self.sampler.visited_waypoints.append((x, y))
            self.node.get_logger().info(
                f"New random waypoint ({x:.2f}, {y:.2f}); "
                f"visited={len(self.sampler.visited_waypoints)}"
            )
            nav.send_goal(x, y, 0.0)
            self.search_phase = "nav"
            self._send_time = time.time()
        return False

    # ==================================================================
    # State 4: navigate to PICK approach.
    # CHANGE vs. Task 2: on goal_succeeded, advance to State 10 (cube
    # pick sub-flow) instead of State 5 (PLACE nav).
    # ==================================================================
    def _tick_state_4_pick(self, approach_nav_timeout):
        nav = self.nav
        if nav.goal_done:
            nav.goal_done = False
            if nav.goal_succeeded:
                self.node.get_logger().info(
                    f"PICK approach reached, starting Task 3 pick of "
                    f"cube ID {self.current_cube_id}"
                )
                self.state = 10
                return True
            else:
                self.node.get_logger().warn(
                    "PICK nav did not succeed, retrying"
                )

        if nav.goal_active:
            if (
                not nav.timeout_fired
                and (time.time() - self._send_time) > approach_nav_timeout
            ):
                self.node.get_logger().warn(
                    "PICK nav timed out, cancelling for retry"
                )
                nav.cancel()
                nav.timeout_fired = True
            return True

        x, y, yaw = self.nav.approach_pose_to_xy_yaw(
            self.aruco.pick_approach_pose
        )
        self.node.get_logger().info(
            f"State 4: nav to PICK approach (cube #{self._current_cube_idx + 1} "
            f"of {len(CUBE_PICK_SEQUENCE)})"
        )
        nav.send_goal(x, y, yaw)
        self._send_time = time.time()
        return False

    # ==================================================================
    # State 5: navigate to PLACE approach.
    # CHANGE vs. Task 2: on goal_succeeded, advance to State 20 (cube
    # drop sub-flow) instead of State 6 (done).
    # ==================================================================
    def _tick_state_5_place(self, approach_nav_timeout):
        nav = self.nav
        if nav.goal_done:
            nav.goal_done = False
            if nav.goal_succeeded:
                self.node.get_logger().info(
                    "PLACE approach reached, starting Task 3 drop"
                )
                self.state = 20
                return True
            else:
                self.node.get_logger().warn(
                    "PLACE nav did not succeed, retrying"
                )

        if nav.goal_active:
            if (
                not nav.timeout_fired
                and (time.time() - self._send_time) > approach_nav_timeout
            ):
                self.node.get_logger().warn(
                    "PLACE nav timed out, cancelling for retry"
                )
                nav.cancel()
                nav.timeout_fired = True
            return True

        x, y, yaw = self.nav.approach_pose_to_xy_yaw(
            self.aruco.place_approach_pose
        )
        self.node.get_logger().info(
            f"State 5: nav to PLACE approach (cube #{self._current_cube_idx + 1})"
        )
        nav.send_goal(x, y, yaw)
        self._send_time = time.time()
        return False

    # ==================================================================
    # State 6: done.
    # ==================================================================
    def _tick_state_6_done(self):
        self.node.get_logger().info("State 6: Task 3 completed")
        self.finished = True

    # ==================================================================
    # Task 3 PICK sub-flow (states 10-18)
    # ==================================================================
    def _tick_state_10_head_tilt(self, detection_timeout):
        # Enable the CubeTracker only NOW: during Task 2 random search
        # the camera could glimpse a cube from far/oblique angle and
        # store a noisy PnP that State 11 would reuse before the head
        # finishes tilting.
        self.cube_tracker.enable()
        # Reset the head scan to the first pan position.
        self._scan_idx = 0
        pan = HEAD_SCAN_PANS[0]
        self.node.get_logger().info(
            f"State 10: tilt={HEAD_TILT_DOWN_FOR_CUBE} rad, "
            f"pan={pan:.2f} rad (scan pos 1/{len(HEAD_SCAN_PANS)})"
        )
        self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, pan)
        self._send_time = time.time()
        self.state = 11

    def _tick_state_11_wait_cube_visible(self, detection_timeout):
        cube_id = self.current_cube_id
        marker = self.cube_tracker.cube_marker_in_map[cube_id]
        if marker is not None:
            self.node.get_logger().info(
                f"Cube ID {cube_id} marker localized at map "
                f"({marker.p.x():.2f}, {marker.p.y():.2f}, {marker.p.z():.2f})"
            )
            # First detection: the wall-marker approach pose typically
            # leaves the cube outside the arm workspace. Navigate to a
            # cube-centric approach pose (CUBE_APPROACH_DISTANCE m
            # straight in front of the cube) before attempting grasp.
            if not self._cube_approached:
                try:
                    self._send_cube_approach_nav(marker)
                except Exception as e:
                    self.node.get_logger().error(
                        f"Failed to compute cube approach nav goal: {e}"
                    )
                    time.sleep(0.5)
                    return
                self.state = 19
                return
            # Second pass (after Nav2 has placed us in front of the
            # cube): closest-observation policy means cube_marker_in_map
            # is now the refined short-range estimate. Compute grasp.
            try:
                self._precompute_grasp_poses(marker)
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to compute grasp poses: {e}"
                )
                time.sleep(0.5)
                return
            self.state = 12
            return
        # Cube not yet detected from this head pan. Dwell HEAD_SCAN_DWELL
        # seconds per pan position; if nothing comes, step the head pan.
        elapsed = time.time() - self._send_time
        if elapsed > HEAD_SCAN_DWELL:
            self._scan_idx += 1
            if self._scan_idx >= len(HEAD_SCAN_PANS):
                self.node.get_logger().warn(
                    f"Full head scan finished without seeing cube ID "
                    f"{cube_id}; restarting scan from center"
                )
                self._scan_idx = 0
            pan = HEAD_SCAN_PANS[self._scan_idx]
            self.node.get_logger().info(
                f"Cube not yet seen, panning head to {pan:.2f} rad "
                f"(scan pos {self._scan_idx + 1}/{len(HEAD_SCAN_PANS)})"
            )
            self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, pan)
            self._send_time = time.time()
            return
        self.node.get_logger().info(
            f"Waiting for cube ID {cube_id} to enter FOV...",
            throttle_duration_sec=2.0,
        )

    def _tick_state_12_open_gripper_pre_grasp(self, gripper_timeout):
        self.node.get_logger().info("State 12: opening gripper before approach")
        self.gripper.open()
        self._send_time = time.time()
        # We do NOT poll completion here -- gripper open is fast and
        # MoveIt2 plans the next arm motion in parallel. Wait a short
        # explicit moment so Gazebo renders the opening clearly (exam
        # requires the gripper motion to be visible).
        time.sleep(1.0)
        self.state = 13

    def _tick_state_13_arm_pre_grasp(self, arm_timeout):
        if not self._arm_goal_sent_for_state(13):
            self.node.get_logger().info(
                "State 13: arm to PRE-GRASP (above cube)"
            )
            self.arm.move_to_pose(
                position=self._pre_grasp_pose_pos,
                quat_xyzw=self._pre_grasp_pose_quat,
            )
            self._mark_arm_goal_sent(13)
            self._send_time = time.time()
            return
        if self.arm.motion_done:
            self.state = 14
            self._clear_arm_goal_sent()
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm pre-grasp timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def _tick_state_14_arm_grasp(self, arm_timeout):
        if not self._arm_goal_sent_for_state(14):
            self.node.get_logger().info(
                "State 14: arm to GRASP (cube center)"
            )
            self.arm.move_to_pose(
                position=self._grasp_pose_pos,
                quat_xyzw=self._grasp_pose_quat,
            )
            self._mark_arm_goal_sent(14)
            self._send_time = time.time()
            return
        if self.arm.motion_done:
            self.state = 15
            self._clear_arm_goal_sent()
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm grasp timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def _tick_state_15_close_gripper(self, gripper_timeout):
        self.node.get_logger().info("State 15: closing gripper around cube")
        self.gripper.close()
        # Visible delay before the attach so the user can see the fingers
        # closing in Gazebo before the link becomes rigid.
        time.sleep(1.0)
        self.state = 16
        self._send_time = time.time()

    def _tick_state_16_attach(self, attach_timeout):
        if not self._arm_goal_sent_for_state(16):
            self.link_attacher.attach(self.current_cube_id)
            self._mark_arm_goal_sent(16)
            self._send_time = time.time()
            return
        if self.link_attacher.action_done:
            if not self.link_attacher.action_succeeded:
                self.node.get_logger().warn(
                    "Attach link failed; retrying attach"
                )
            self.state = 17
            self._clear_arm_goal_sent()
            return
        if (time.time() - self._send_time) > attach_timeout:
            self.node.get_logger().warn("Attach service timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def _tick_state_17_arm_lift_after_grasp(self, arm_timeout):
        if not self._arm_goal_sent_for_state(17):
            self.node.get_logger().info(
                "State 17: lifting arm (back to pre-grasp height)"
            )
            self.arm.move_to_pose(
                position=self._pre_grasp_pose_pos,
                quat_xyzw=self._pre_grasp_pose_quat,
            )
            self._mark_arm_goal_sent(17)
            self._send_time = time.time()
            return
        if self.arm.motion_done:
            self.state = 18
            self._clear_arm_goal_sent()
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm lift timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def _tick_state_18_arm_carry(self, arm_timeout):
        # Tuck arm to HOME so navigation isn't obstructed by an
        # extended arm. The cube stays attached -- HOME keeps the
        # gripper inside the robot's footprint while carrying.
        if not self._arm_goal_sent_for_state(18):
            self.node.get_logger().info(
                "State 18: tucking arm to HOME for navigation"
            )
            self.arm.move_to_home(tilt_head=False)
            self._mark_arm_goal_sent(18)
            self._send_time = time.time()
            return
        if self.arm.motion_done:
            self.node.get_logger().info("Carry config ready -> nav to PLACE")
            self._clear_arm_goal_sent()
            # Make sure nav is in a clean state before state 5.
            self.nav.goal_active = False
            self.nav.goal_done = False
            self.nav.goal_succeeded = False
            self.nav.timeout_fired = False
            self.state = 5
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm tuck timed out, retrying")
            self._clear_arm_goal_sent()
            return

    # ==================================================================
    # State 19: navigate from the wall-marker approach pose to a refined
    # cube-centric approach pose (CUBE_APPROACH_DISTANCE m straight in
    # front of the cube, base facing the cube). Triggered once per cube
    # from State 11 on first detection.
    # ==================================================================
    def _tick_state_19_wait_cube_nav(self, approach_nav_timeout):
        nav = self.nav
        if nav.goal_done:
            nav.goal_done = False
            if nav.goal_succeeded:
                self.node.get_logger().info(
                    "Cube approach Nav2 reached (within inflation limit); "
                    "pushing remaining gap with /drive_on_heading"
                )
                # Re-tilt head straight down so the closer detection is
                # captured while we push the base forward.
                self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, 0.0)
                time.sleep(1.0)
                self._send_drive_on_heading_to_cube()
                self.state = 27
                return
            self.node.get_logger().warn(
                "Cube approach nav did not succeed; attempting grasp anyway"
            )
            self._cube_approached = True
            self.state = 11
            return
        if nav.goal_active:
            if (
                not nav.timeout_fired
                and (time.time() - self._send_time) > approach_nav_timeout
            ):
                self.node.get_logger().warn(
                    "Cube approach nav timed out, cancelling"
                )
                nav.cancel()
                nav.timeout_fired = True
            return

    # ==================================================================
    # State 27: wait for the /drive_on_heading push that bridges the gap
    # left by Nav2's inflation_radius. On success we hand control back
    # to State 11 to compute grasp poses from the refined cube position.
    # ==================================================================
    def _tick_state_27_wait_drive_on_heading(self):
        nav = self.nav
        DRIVE_TIMEOUT = 25.0
        if nav.drive_done:
            nav.drive_done = False
            if nav.drive_succeeded:
                self.node.get_logger().info(
                    "drive_on_heading finished, robot now in arm range"
                )
            else:
                self.node.get_logger().warn(
                    "drive_on_heading did not succeed; attempting grasp anyway"
                )
            self._cube_approached = True
            # Brief settle so the cube tracker has fresh detections at
            # the new (closer) range before _precompute_grasp_poses runs.
            time.sleep(1.5)
            self.state = 11
            return
        if nav.drive_active:
            if (time.time() - self._send_time) > DRIVE_TIMEOUT:
                self.node.get_logger().warn(
                    "drive_on_heading timed out, cancelling"
                )
                nav.cancel_drive()
            return

    def _send_drive_on_heading_to_cube(self):
        # Compute how much further forward the base must travel to end
        # up CUBE_APPROACH_DISTANCE m from the cube. The drive_on_heading
        # action drives in a straight line along the robot's current X
        # axis (the State 19 nav set yaw to face the cube, so X = cube
        # direction).
        cube_id = self.current_cube_id
        marker = self.cube_tracker.cube_marker_in_map[cube_id]
        if marker is None:
            self.node.get_logger().warn(
                "drive_on_heading skipped: cube marker missing"
            )
            self.nav.drive_done = True
            self.nav.drive_succeeded = False
            return
        cube_x = marker.p.x()
        cube_y = marker.p.y()
        try:
            tf_map_base = self.tf_buffer.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            self.node.get_logger().warn(
                f"drive_on_heading skipped: TF lookup failed: {e}"
            )
            self.nav.drive_done = True
            self.nav.drive_succeeded = False
            return
        rx = tf_map_base.transform.translation.x
        ry = tf_map_base.transform.translation.y
        current_dist = math.hypot(cube_x - rx, cube_y - ry)
        push = current_dist - CUBE_APPROACH_DISTANCE
        if push <= 0.02:
            self.node.get_logger().info(
                f"Already at target distance ({current_dist:.2f} m), "
                "skipping drive_on_heading"
            )
            self.nav.drive_done = True
            self.nav.drive_succeeded = True
            return
        self.node.get_logger().info(
            f"drive_on_heading: current dist {current_dist:.2f} m, "
            f"target {CUBE_APPROACH_DISTANCE:.2f} m, pushing {push:.2f} m"
        )
        self.nav.send_drive_on_heading(push, speed=DRIVE_SPEED)
        self._send_time = time.time()

    def _send_cube_approach_nav(self, marker_in_map):
        # Builds a Nav2 goal that puts base_link CUBE_APPROACH_DISTANCE
        # meters from the cube center, perpendicular to the PICK wall
        # (i.e. along the wall marker normal direction). We do NOT use
        # the robot->cube direction because it can park the robot at
        # the SIDE of the pick surface, where the footprint enters the
        # surface's inscribed_radius zone and DWB rejects every
        # trajectory as "BaseObstacle/Trajectory Hits Obstacle".
        cube_x = marker_in_map.p.x()
        cube_y = marker_in_map.p.y()

        # Extract the wall-approach yaw (perpendicular to the pick wall,
        # facing it) from the latched pick_approach_pose. PnP error on
        # this yaw is typically <10 deg, more reliable than the robot-
        # to-cube heading which is affected by AMCL drift + Nav2 xy_tol.
        pick_approach = self.aruco.pick_approach_pose
        qx = pick_approach.pose.orientation.x
        qy = pick_approach.pose.orientation.y
        qz = pick_approach.pose.orientation.z
        qw = pick_approach.pose.orientation.w
        wall_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        # Approach pose: CUBE_APPROACH_DISTANCE m back from the cube
        # along the wall normal, with yaw = wall_yaw so the robot ends
        # up frontal to the wall (cube ~in front of robot).
        ax = cube_x - CUBE_APPROACH_DISTANCE * math.cos(wall_yaw)
        ay = cube_y - CUBE_APPROACH_DISTANCE * math.sin(wall_yaw)
        yaw = wall_yaw

        # Diagnostic: current robot->cube distance (just for the log,
        # not used in the math).
        try:
            tf_map_base = self.tf_buffer.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            rx = tf_map_base.transform.translation.x
            ry = tf_map_base.transform.translation.y
            d = math.hypot(cube_x - rx, cube_y - ry)
        except Exception:
            d = float("nan")

        self.node.get_logger().info(
            f"Cube is {d:.2f} m away (too far for arm); sending Nav2 "
            f"to cube approach pose ({ax:.2f}, {ay:.2f}) "
            f"yaw={yaw:.2f} (wall-normal)"
        )
        # Reset nav latches before sending the new goal.
        self.nav.goal_active = False
        self.nav.goal_done = False
        self.nav.goal_succeeded = False
        self.nav.timeout_fired = False
        self.nav.send_goal(ax, ay, yaw)
        self._send_time = time.time()

    # ==================================================================
    # Task 3 DROP sub-flow (states 20-26)
    # ==================================================================
    def _tick_state_20_arm_pre_drop(self, arm_timeout):
        if not self._arm_goal_sent_for_state(20):
            try:
                self._precompute_drop_poses()
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to compute drop poses: {e}"
                )
                time.sleep(0.5)
                return
            self.node.get_logger().info(
                "State 20: arm to PRE-DROP (above place surface)"
            )
            self.arm.move_to_pose(
                position=self._pre_drop_pose_pos,
                quat_xyzw=self._pre_drop_pose_quat,
            )
            self._mark_arm_goal_sent(20)
            self._send_time = time.time()
            return
        if self.arm.motion_done:
            self.state = 21
            self._clear_arm_goal_sent()
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm pre-drop timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def _tick_state_21_arm_drop(self, arm_timeout):
        if not self._arm_goal_sent_for_state(21):
            self.node.get_logger().info(
                "State 21: arm to DROP (just above surface)"
            )
            self.arm.move_to_pose(
                position=self._drop_pose_pos,
                quat_xyzw=self._drop_pose_quat,
            )
            self._mark_arm_goal_sent(21)
            self._send_time = time.time()
            return
        if self.arm.motion_done:
            self.state = 22
            self._clear_arm_goal_sent()
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm drop timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def _tick_state_22_open_gripper_release(self, gripper_timeout):
        self.node.get_logger().info("State 22: opening gripper to release cube")
        self.gripper.open()
        # Visible delay so the gripper opening renders before detach.
        time.sleep(1.0)
        self.state = 23
        self._send_time = time.time()

    def _tick_state_23_detach(self, attach_timeout):
        if not self._arm_goal_sent_for_state(23):
            self.link_attacher.detach(self.current_cube_id)
            self._mark_arm_goal_sent(23)
            self._send_time = time.time()
            return
        if self.link_attacher.action_done:
            if not self.link_attacher.action_succeeded:
                self.node.get_logger().warn(
                    "Detach link failed; continuing anyway"
                )
            self.state = 24
            self._clear_arm_goal_sent()
            return
        if (time.time() - self._send_time) > attach_timeout:
            self.node.get_logger().warn("Detach service timed out")
            self.state = 24
            self._clear_arm_goal_sent()
            return

    def _tick_state_24_arm_lift_after_drop(self, arm_timeout):
        if not self._arm_goal_sent_for_state(24):
            self.node.get_logger().info(
                "State 24: lifting arm clear of dropped cube"
            )
            self.arm.move_to_pose(
                position=self._pre_drop_pose_pos,
                quat_xyzw=self._pre_drop_pose_quat,
            )
            self._mark_arm_goal_sent(24)
            self._send_time = time.time()
            return
        if self.arm.motion_done:
            self.state = 25
            self._clear_arm_goal_sent()
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm lift-after-drop timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def _tick_state_25_arm_carry_back(self, arm_timeout):
        if not self._arm_goal_sent_for_state(25):
            self.node.get_logger().info(
                "State 25: tucking arm to HOME for nav back"
            )
            self.arm.move_to_home(tilt_head=False)
            self._mark_arm_goal_sent(25)
            self._send_time = time.time()
            return
        if self.arm.motion_done:
            self._clear_arm_goal_sent()
            self.state = 26
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm tuck-back timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def _tick_state_26_next_or_done(self):
        finished_cube_id = self.current_cube_id
        self.node.get_logger().info(
            f"State 26: cube ID {finished_cube_id} placed"
        )
        self._current_cube_idx += 1
        if self._current_cube_idx >= len(CUBE_PICK_SEQUENCE):
            self.node.get_logger().info(
                "All cubes placed -> Task 3 done"
            )
            self.state = 6
            return
        # Next cube: clean nav state, reset the cube tracker entry for
        # the upcoming marker (in case a stale long-range observation
        # was kept while we were busy with the previous cube), then go
        # back to State 4 (re-nav to PICK approach).
        next_id = self.current_cube_id
        self.node.get_logger().info(
            f"Next cube to pick: ID {next_id} -> nav back to PICK"
        )
        self.cube_tracker.reset_cube(next_id)
        # Reset the cube-approach flag so the next cube also triggers
        # its own short-range nav refinement in State 11.
        self._cube_approached = False
        self.nav.goal_active = False
        self.nav.goal_done = False
        self.nav.goal_succeeded = False
        self.nav.timeout_fired = False
        self.state = 4

    # ==================================================================
    # Helpers
    # ==================================================================
    def _precompute_grasp_poses(self, marker_in_map):
        # Cube center in map: marker sits on +Z face of cube, so cube
        # center is CUBE_TOP_TO_CENTER below the marker frame origin.
        cube_x = marker_in_map.p.x()
        cube_y = marker_in_map.p.y()
        cube_center_z = marker_in_map.p.z() - CUBE_TOP_TO_CENTER

        # Robot position in map (the cube marker on top has Z=up so it
        # gives no useful yaw; we derive yaw from the robot heading so
        # fingers open perpendicular to the approach direction).
        tf_map_base = self.tf_buffer.lookup_transform(
            "map", "base_link",
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        robot_x = tf_map_base.transform.translation.x
        robot_y = tf_map_base.transform.translation.y
        yaw_to_cube = math.atan2(cube_y - robot_y, cube_x - robot_x)

        # Top-down grasp matching Lab 3 "modify approach_frame to grasp
        # from the top of the marker" hint. The cube marker frame has
        # Z=up; DoRotY(pi/2) on it gives X=down (approach axis), which
        # in absolute map coords is RPY(0, pi/2, yaw):
        #   X_gripper → (0, 0, -1)              — pointing straight down
        #   Y_gripper → (-sin(yaw), cos(yaw), 0) — horizontal, perp. to approach
        #   Z_gripper → (cos(yaw), sin(yaw), 0) — horizontal, aligned with approach
        # gripper_grasping_frame Z = gripper_link Y = finger opening axis,
        # so fingers open along the approach line (one on the near face
        # of the cube, one on the far face). Both fingers stay at the
        # SAME height = cube center, well above the surface.
        yaw_to_cube_rotated = yaw_to_cube + (math.pi / 2.0)
        grasp_orient_in_map = Rotation.RPY(0.0, math.pi / 2.0, yaw_to_cube_rotated)

        # Grasp: gripper_grasping_frame at the cube center.
        grasp_z = cube_center_z + CUBE_TOP_TO_CENTER + 0.005
        grasp_frame_in_map = Frame(
            grasp_orient_in_map, Vector(cube_x, cube_y, grasp_z)
        )
        # Pre-grasp: PRE_GRASP_LIFT meters straight ABOVE the cube
        # (gripper hovers, then descends to the grasp pose).
        pre_grasp_frame_in_map = Frame(
            grasp_orient_in_map,
            Vector(cube_x, cube_y, cube_center_z + PRE_GRASP_LIFT),
        )

        # MoveIt2 plans in base_link (configured in tiago_arm.py).
        # Transform both poses from map -> base_link.
        grasp_in_base = self._transform_to_base_link(grasp_frame_in_map)
        pre_grasp_in_base = self._transform_to_base_link(pre_grasp_frame_in_map)

        self._grasp_pose_pos, self._grasp_pose_quat = self._frame_to_pos_quat(
            grasp_in_base
        )
        self._pre_grasp_pose_pos, self._pre_grasp_pose_quat = (
            self._frame_to_pos_quat(pre_grasp_in_base)
        )
        self.node.get_logger().info(
            f"[diag] cube_center=({cube_x:.2f}, {cube_y:.2f}, "
            f"{cube_center_z:.2f}) yaw={yaw_to_cube:.2f}; "
            f"pre_grasp_base=({self._pre_grasp_pose_pos[0]:.2f}, "
            f"{self._pre_grasp_pose_pos[1]:.2f}, "
            f"{self._pre_grasp_pose_pos[2]:.2f})"
        )

    def _precompute_drop_poses(self):
        # We don't use a cube marker for the drop -- the cube IS in the
        # gripper. The target is "in front of the PLACE approach pose, on
        # top of the place surface", which we infer from the latched
        # PLACE approach PoseStamped (set by ArucoTracker on wall marker
        # ID 238) plus the geometry constants.
        approach = self.aruco.place_approach_pose
        ax = approach.pose.position.x
        ay = approach.pose.position.y
        # Approach yaw points TOWARD the marker (we set it that way in
        # task2_aruco._compute_horizontal_approach).
        qx = approach.pose.orientation.x
        qy = approach.pose.orientation.y
        qz = approach.pose.orientation.z
        qw = approach.pose.orientation.w
        # 2D yaw from quaternion.
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))

        # Drop XY = forward of approach pose along its yaw.
        drop_x = ax + PLACE_FORWARD_OFFSET * math.cos(yaw)
        drop_y = ay + PLACE_FORWARD_OFFSET * math.sin(yaw)

        # Same top-down orientation as for grasping. yaw is the robot's
        # heading at the PLACE approach pose (already toward the wall).
        drop_orient_in_map = Rotation.RPY(0.0, math.pi / 2.0, yaw)
        drop_frame_in_map = Frame(
            drop_orient_in_map, Vector(drop_x, drop_y, PLACE_TARGET_Z)
        )
        # Pre-drop: POST_GRASP_LIFT meters straight ABOVE the drop point.
        pre_drop_frame_in_map = Frame(
            drop_orient_in_map,
            Vector(drop_x, drop_y, PLACE_TARGET_Z + POST_GRASP_LIFT),
        )
        drop_in_base = self._transform_to_base_link(drop_frame_in_map)
        pre_drop_in_base = self._transform_to_base_link(pre_drop_frame_in_map)

        self._drop_pose_pos, self._drop_pose_quat = self._frame_to_pos_quat(
            drop_in_base
        )
        self._pre_drop_pose_pos, self._pre_drop_pose_quat = (
            self._frame_to_pos_quat(pre_drop_in_base)
        )

    def _transform_to_base_link(self, frame_in_map: Frame) -> Frame:
        # Use the latest map<-base_link TF (we don't have a per-detection
        # stamp here -- the robot is parked at the approach pose so the
        # TF doesn't move during planning).
        tf_base_map = self.tf_buffer.lookup_transform(
            "base_link", "map",
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        frame_base_map = transform_to_kdl_frame(tf_base_map)
        return frame_base_map * frame_in_map

    @staticmethod
    def _frame_to_pos_quat(frame: Frame):
        pos = [frame.p.x(), frame.p.y(), frame.p.z()]
        qx, qy, qz, qw = frame.M.GetQuaternion()
        # pymoveit2.move_to_pose expects quat in xyzw order.
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        quat = [qx / n, qy / n, qz / n, qw / n]
        return pos, quat

    def _arm_goal_sent_for_state(self, state_id: int) -> bool:
        return getattr(self, "_arm_goal_for_state", None) == state_id

    def _mark_arm_goal_sent(self, state_id: int):
        self._arm_goal_for_state = state_id

    def _clear_arm_goal_sent(self):
        self._arm_goal_for_state = None
