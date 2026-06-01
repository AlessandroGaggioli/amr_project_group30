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
from tiago_project_group30.task2_constants import CAMERA_FRAME
from tiago_project_group30.task3_constants import (
    CUBE_APPROACH_DISTANCE,
    SAFE_NAV_DISTANCE,
    CUBE_PICK_SEQUENCE,
    CUBE_TOP_TO_CENTER,
    DRIVE_SPEED,
    GRASP_Z_ABOVE_TOP,
    HEAD_SCAN_DWELL,
    HEAD_SCAN_PANS,
    HEAD_TILT_DOWN_FOR_CUBE,
    PLACE_APPROACH_DISTANCE,
    PLACE_FORWARD_OFFSET,
    PLACE_LATERAL_OFFSET,
    PLACE_TARGET_Z,
    POST_GRASP_LIFT,
    PRE_GRASP_LIFT,
)
from tiago_project_group30.common_states import CommonStates


class Task3StateMachine(CommonStates):
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
        # "pick" or "place": selects the reference frame and stop distance
        # used by the shared push (State 27) and back_out (State 28) states.
        self._push_mode = "pick"

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
        SEARCH_NAV_TIMEOUT = 120.0
        APPROACH_NAV_TIMEOUT = 180.0
        # Task 3-side timeouts.
        ARM_MOTION_TIMEOUT = 20.0       # MoveIt2 plan+execute per arm goal
        GRIPPER_TIMEOUT = 5.0
        ATTACH_TIMEOUT = 3.0
        CUBE_DETECTION_TIMEOUT = 15.0   # max wait for cube marker after head tilt

        while rclpy.ok() and not self.finished:
            try:
                self.arm.update_flags()
                self.nav.update_flags()
                self.amcl.update_spin_flags()
                self.amcl.update_amcl_flags()
                self.gripper.update_flags()

                if self.state == 0:
                    self.state_0_arm_tuck()
                elif self.state == 1:
                    self.state_1_wait_arm(PLANNING_TIMEOUT)
                elif self.state == 2:
                    self.state_2_amcl_localization()
                elif self.state == 3:
                    if self.state_3_random_search(SEARCH_NAV_TIMEOUT):
                        continue
                elif self.state == 4:
                    if self.state_4_pick(APPROACH_NAV_TIMEOUT):
                        continue
                elif self.state == 5:
                    if self.state_5_place(APPROACH_NAV_TIMEOUT):
                        continue
                elif self.state == 6:
                    self.state_6_done()
                    break

                # ---- Task 3 pick sub-flow ----
                elif self.state == 10:
                    self.state_10_head_tilt(CUBE_DETECTION_TIMEOUT)
                elif self.state == 11:
                    self.state_11_wait_cube_visible(CUBE_DETECTION_TIMEOUT)
                elif self.state == 12:
                    self.state_12_open_gripper_pre_grasp(GRIPPER_TIMEOUT)
                elif self.state == 13:
                    self.state_13_arm_pre_grasp(ARM_MOTION_TIMEOUT)
                elif self.state == 14:
                    self.state_14_arm_grasp(ARM_MOTION_TIMEOUT)
                elif self.state == 15:
                    self.state_15_close_gripper(GRIPPER_TIMEOUT)
                elif self.state == 16:
                    self.state_16_attach(ATTACH_TIMEOUT)
                elif self.state == 17:
                    self.state_17_arm_lift_after_grasp(ARM_MOTION_TIMEOUT)
                elif self.state == 18:
                    self.state_18_arm_carry(ARM_MOTION_TIMEOUT)
                elif self.state == 19:
                    self.state_19_wait_cube_nav(APPROACH_NAV_TIMEOUT)

                # ---- Task 3 drop sub-flow ----
                elif self.state == 20:
                    self.state_20_arm_pre_drop(ARM_MOTION_TIMEOUT)
                elif self.state == 21:
                    self.state_21_arm_drop(ARM_MOTION_TIMEOUT)
                elif self.state == 22:
                    self.state_22_open_gripper_release(GRIPPER_TIMEOUT)
                elif self.state == 23:
                    self.state_23_detach(ATTACH_TIMEOUT)
                elif self.state == 24:
                    self.state_24_arm_lift_after_drop(ARM_MOTION_TIMEOUT)
                elif self.state == 25:
                    self.state_25_arm_carry_back(ARM_MOTION_TIMEOUT)
                elif self.state == 26:
                    self.state_26_next_or_done()
                elif self.state == 27:
                    self.state_27_cmd_vel_push()
                elif self.state == 28:
                    self.state_28_back_out()
                elif self.state == 29:
                    self.state_29_refresh_place_marker(CUBE_DETECTION_TIMEOUT)

            except Exception as e:
                import traceback
                self.node.get_logger().error(
                    f"Exception in state machine: {e}\n{traceback.format_exc()}"
                )

            time.sleep(0.05)

    

    # ==================================================================
    # State 4: navigate to PICK approach.
    # CHANGE vs. Task 2: on goal_succeeded, advance to State 10 (cube
    # pick sub-flow) instead of State 5 (PLACE nav).
    # ==================================================================
    def state_4_pick(self, approach_nav_timeout):
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
    def state_5_place(self, approach_nav_timeout):
        nav = self.nav
        if nav.goal_done:
            nav.goal_done = False
            if nav.goal_succeeded:
                self.node.get_logger().info(
                    "PLACE approach reached, re-detecting marker up close"
                )
                # State 29 refreshes the (far-frozen) place marker with a
                # close-range detection, then hands off to the push (State 27,
                # "place" mode) and the drop (State 20).
                self._send_time = time.time()
                self.state = 29
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
    def state_6_done(self):
        self.node.get_logger().info("State 6: Task 3 completed")
        self.finished = True

    # ==================================================================
    # Task 3 PICK sub-flow (states 10-18)
    # ==================================================================
    def state_10_head_tilt(self, detection_timeout):
        # Enable the CubeTracker only NOW: during Task 2 random search
        # the camera could glimpse a cube from far/oblique angle and
        # store a noisy PnP that State 11 would reuse before the head
        # finishes tilting.
        self.cube_tracker.enable()
        # Single tilt straight down, no pan (no scan). The wall-marker
        # approach already aligns the robot with the surface, so the
        # cube should be in front; the strict TF-sync filter in the
        # cube tracker will drop the first few noisy detections during
        # head motion and accept the first synced one once the head
        # settles.
        self.node.get_logger().info(
            f"State 10: tilt={HEAD_TILT_DOWN_FOR_CUBE} rad, pan=0.0 rad"
        )
        self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, 0.0)
        self._send_time = time.time()
        self._head_scan_active = False
        self._head_scan_idx = 0
        self._head_scan_time = None
        self.state = 11

    def state_11_wait_cube_visible(self, detection_timeout):
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
            # is now the refined short-range estimate. FREEZE it so a late
            # noisy detection can't shift the target sideways during the
            # grasp, then compute grasp against the latched pose.
            self.cube_tracker.freeze()
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
        # No marker yet: start a head scan only when the cube is not in
        # the FOV. This keeps the head still for a brief window so any
        # immediate detection isn't invalidated by motion.
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

        if (time.time() - self._head_scan_time) >= HEAD_SCAN_DWELL:
            self._head_scan_idx = (self._head_scan_idx + 1) % len(HEAD_SCAN_PANS)
            pan = HEAD_SCAN_PANS[self._head_scan_idx]
            self.node.get_logger().info(
                f"Head scan pan={pan:.2f} rad",
                throttle_duration_sec=1.0,
            )
            self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, pan)
            self._head_scan_time = time.time()

    def state_12_open_gripper_pre_grasp(self, gripper_timeout):
        self.node.get_logger().info("State 12: opening gripper before approach")
        self.gripper.open()
        self._send_time = time.time()
        # We do NOT poll completion here -- gripper open is fast and
        # MoveIt2 plans the next arm motion in parallel. Wait a short
        # explicit moment so Gazebo renders the opening clearly (exam
        # requires the gripper motion to be visible).
        time.sleep(1.0)
        self.state = 13

    def state_13_arm_pre_grasp(self, arm_timeout):
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

    def state_14_arm_grasp(self, arm_timeout):
        if not self._arm_goal_sent_for_state(14):
            self.node.get_logger().info(
                "State 14: arm to GRASP (straight-line cartesian descent)"
            )
            # Cartesian (straight-line) descent from the pre-grasp hover to
            # the grasp pose: the gripper drops STRAIGHT DOWN onto the cube.
            # A joint-space plan here curves the end-effector sideways and
            # the open fingers clip the cube on the way down.
            self.arm.move_to_pose(
                position=self._grasp_pose_pos,
                quat_xyzw=self._grasp_pose_quat,
                cartesian=True,
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

    def state_15_close_gripper(self, gripper_timeout):
        # WAIT for the fingers to FINISH closing before advancing to the
        # attach (State 16). The old version fired gripper.close() then
        # slept a fixed 1.0 s and moved on -- but the close takes ~3.6 s in
        # Gazebo, so the IFRA attach rigidly bolted the cube to the finger
        # while the fingers were still mid-travel and not yet settled on
        # the cube. The fixed-joint constraint then fought the prismatic
        # finger joint, Gazebo blew up the contact forces (the cube
        # "glitching"), and because the cube is now rigidly tied to the
        # robot those forces propagated into the base, making it thrash.
        # Polling action_done lets the grip settle physically first.
        if not self._arm_goal_sent_for_state(15):
            self.node.get_logger().info("State 15: closing gripper around cube")
            self.gripper.close()
            self._mark_arm_goal_sent(15)
            self._send_time = time.time()
            return
        if self.gripper.action_done:
            # Brief settle so the contact stabilises before the link
            # becomes rigid.
            time.sleep(0.5)
            self.state = 16
            self._clear_arm_goal_sent()
            self._send_time = time.time()
            return
        if (time.time() - self._send_time) > gripper_timeout:
            self.node.get_logger().warn(
                "Gripper close timed out; attaching anyway"
            )
            self.state = 16
            self._clear_arm_goal_sent()
            self._send_time = time.time()
            return

    def state_16_attach(self, attach_timeout):
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

    def state_17_arm_lift_after_grasp(self, arm_timeout):
        if not self._arm_goal_sent_for_state(17):
            self.node.get_logger().info(
                "State 17: lifting arm (straight-line cartesian lift)"
            )
            # Cartesian (straight-line) lift, the inverse of the State 14
            # descent: the gripper rises STRAIGHT UP from the grasp pose to
            # the pre-grasp hover. A joint-space plan would swing the just-
            # grasped cube sideways and could drag it against the cube next
            # to it or the table edge.
            self.arm.move_to_pose(
                position=self._pre_grasp_pose_pos,
                quat_xyzw=self._pre_grasp_pose_quat,
                cartesian=True,
            )
            self._mark_arm_goal_sent(17)
            self._send_time = time.time()
            return
        self._log_gripper_z("State17-lift")
        if self.arm.motion_done:
            self.state = 18
            self._clear_arm_goal_sent()
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm lift timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def _log_gripper_z(self, tag):
        # Diagnostic: log the gripper_grasping_frame height (and the
        # fingertip height ~0.10 m below it along the downward approach)
        # in map while the arm carries the attached cube, so we can see
        # whether the cube/fingers dip toward the table during lift/tuck.
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

    def state_18_arm_carry(self, arm_timeout):
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
        self._log_gripper_z("State18-tuck")
        if self.arm.motion_done:
            self.node.get_logger().info(
                "Carry config ready -> backing out of pick zone"
            )
            # Raise the head back to level: it was tilted down to see the
            # cube, and a level head looks tidier (and frees the depth FOV)
            # for the drive to PLACE. Fire-and-forget on the head controller.
            self.arm.tilt_head(0.0, 0.0)
            self._clear_arm_goal_sent()
            # State 27's forward push parked us ~CUBE_APPROACH_DISTANCE m
            # from the cube -- INSIDE the pick surface's inflation_radius.
            # Planning the PLACE route from there fails and Nav2 thrashes
            # through recovery. back_out (State 28, "pick" mode) reverses to
            # SAFE_NAV_DISTANCE first, then hands off to the PLACE nav.
            self._push_mode = "pick"
            self._send_time = time.time()
            self.state = 28
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
    def state_19_wait_cube_nav(self, approach_nav_timeout):
        nav = self.nav
        if nav.goal_done:
            nav.goal_done = False
            if nav.goal_succeeded:
                self.node.get_logger().info(
                    "Cube approach Nav2 reached (within inflation limit); "
                    "closing remaining gap with a /cmd_vel push"
                )
                # Re-tilt head straight down so the closer detection is
                # captured while we push the base forward.
                self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, 0.0)
                time.sleep(1.0)
                self._push_mode = "pick"
                self._send_time = time.time()
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
    # State 27: closed-loop /cmd_vel push toward a reference frame,
    # steering frontal, used for BOTH the pick and the place approach
    # (selected by self._push_mode). Nav2's inflation_radius leaves the
    # base parked too far from the target; this drives the remaining gap
    # while nulling the heading error so the robot arrives frontal.
    #   pick  -> toward the cube marker, stop within CUBE_APPROACH_DISTANCE,
    #            then grasp (State 11).
    #   place -> toward the place wall marker, stop within
    #            PLACE_APPROACH_DISTANCE, then drop (State 20).
    # ==================================================================
    # def state_27_cmd_vel_push(self):
    #     is_pick = self._push_mode != "place"
    #     ref = self._push_reference_xy()
    #     if ref is None:
    #         self.node.get_logger().warn(
    #             f"push ({self._push_mode}): reference marker missing, "
    #             "proceeding anyway"
    #         )
    #         self.nav.stop()
    #         if is_pick:
    #             self._cube_approached = True
    #             self.state = 11
    #         else:
    #             self.state = 20
    #         return

    #     stop_dist = CUBE_APPROACH_DISTANCE if is_pick else PLACE_APPROACH_DISTANCE
    #     result = self._cmd_vel_push(
    #         ref[0], ref[1], stop_dist=stop_dist,
    #         timeout=40.0, steer=True, tag=f"push ({self._push_mode})",
    #     )
    #     if result is not None:
    #         if is_pick:
    #             self._cube_approached = True
    #             if result == "done":
    #                 # Brief settle so the cube tracker has fresh close-range
    #                 # detections before _precompute_grasp_poses runs.
    #                 time.sleep(1.5)
    #             self.state = 11
    #         else:
    #             self.state = 20

    def state_27_cmd_vel_push(self):
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
                self.state = 11
            else:
                self.state = 20
            return

        stop_dist = CUBE_APPROACH_DISTANCE if is_pick else PLACE_APPROACH_DISTANCE
        result = self._cmd_vel_push(
            ref[0], ref[1], stop_dist=stop_dist,
            timeout=40.0, steer=True, tag=f"push ({self._push_mode})",
        )
        
        if result is not None:
            if is_pick:
                self._cube_approached = True
                if result == "done" or result == "timeout":

                    # Pieghiamo la testa leggermente più giù per centrare il marker 
                    # da vicino e avere una lettura ottica perfetta.
                    self.arm.tilt_head(-1.15, 0.0)
                    # Pausa per fermare l'inerzia del robot
                    time.sleep(1.5)
                    
                    # --- FIX CRUCIALE ---
                    # 1. Cancelliamo i dati registrati durante il movimento!
                    self.cube_tracker.reset_cube(self.current_cube_id)
                    self.cube_tracker.unfreeze()

                    # --------------------
                    
                self.state = 11
            else:
                self.state = 20

    # ==================================================================
    # State 28: back_out -- straight reverse, the mirror of State 27.
    # Reverses away from the reference frame (self._push_mode) until
    # base_link is at least SAFE_NAV_DISTANCE from it, clearing the
    # surface's inflation_radius so Nav2 can plan the next leg. Used
    #   pick  -> after grasp/tuck, then -> nav to PLACE (State 5).
    #   place -> after drop/tuck, then -> next cube or done (State 26).
    # ==================================================================
    def state_28_back_out(self):
        BACK_TIMEOUT = 60.0
        next_state = 5 if self._push_mode != "place" else 26

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
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            self.node.get_logger().warn(
                f"back_out ({self._push_mode}): TF lookup failed ({e}); stopping",
                throttle_duration_sec=2.0,
            )
            self.nav.stop()
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        dist = math.hypot(ref[0] - rx, ref[1] - ry)

        # Far enough from the surface -> stop and go to the next leg.
        if dist >= (SAFE_NAV_DISTANCE+0.5):
            self.nav.stop()
            self.node.get_logger().info(
                f"back_out ({self._push_mode}) done: {dist:.2f} m from ref "
                f"(target {SAFE_NAV_DISTANCE:.2f} m), clear of inflation"
            )
            # Clean nav latches before handing back to a Nav2 leg.
            self.nav.goal_active = False
            self.nav.goal_done = False
            self.nav.goal_succeeded = False
            self.nav.timeout_fired = False
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

        # Still inside the zone -> keep reversing (negative speed).
        self.nav.publish_forward(-DRIVE_SPEED)
        self.node.get_logger().info(
            f"back_out ({self._push_mode}): {dist:.2f} m -> target "
            f"{SAFE_NAV_DISTANCE:.2f} m",
            throttle_duration_sec=1.0,
        )

    def state_29_refresh_place_marker(self, timeout):
        # The place marker was localized from far away during the search and
        # then frozen, so the push and the drop both targeted a wrong point
        # (cube missed the table / wasn't centred). Now that we're parked in
        # front of the surface, tilt the head down to the (low) marker and let
        # a FRESH close-range detection overwrite place_marker_in_map before
        # the push, exactly like the camera anti-drift used for the cube.
        if not self._arm_goal_sent_for_state(29):
            self.node.get_logger().info(
                "State 29: re-detecting place marker up close before drop"
            )
            # Un-freeze + reset KEEP-CLOSEST so the close detection wins over
            # the stale far-away one.
            self.aruco.place_detection_distance = float("inf")
            self.aruco._refresh_approach_poses = True
            # Start a head scan (tilt down + pan sweep) to bring the low place
            # marker into the FOV, same as the pick cube search (State 11):
            # the robot rarely parks dead-centre on the surface, so a fixed
            # pan=0 can miss the marker.
            self._head_scan_idx = 0
            self._head_scan_time = time.time()
            self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, HEAD_SCAN_PANS[0])
            self._mark_arm_goal_sent(29)
            self._send_time = time.time()
            return

        # A close detection has landed once the stored distance is small
        # (we're ~1 m from the marker here).
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
            self.state = 27
            return

        # Not refreshed yet -> keep sweeping the head across the pan positions.
        if (time.time() - self._head_scan_time) >= HEAD_SCAN_DWELL:
            self._head_scan_idx = (self._head_scan_idx + 1) % len(HEAD_SCAN_PANS)
            pan = HEAD_SCAN_PANS[self._head_scan_idx]
            self.node.get_logger().info(
                f"Place marker scan pan={pan:.2f} rad",
                throttle_duration_sec=1.0,
            )
            self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, pan)
            self._head_scan_time = time.time()

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
        # ax = cube_x - CUBE_APPROACH_DISTANCE * math.cos(wall_yaw)
        # ay = cube_y - CUBE_APPROACH_DISTANCE * math.sin(wall_yaw)
        # yaw = wall_yaw

        yaw = wall_yaw
        ax = cube_x - SAFE_NAV_DISTANCE * math.cos(yaw)
        ay = cube_y - SAFE_NAV_DISTANCE * math.sin(yaw)


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
    def state_20_arm_pre_drop(self, arm_timeout):
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

    def state_21_arm_drop(self, arm_timeout):
        if not self._arm_goal_sent_for_state(21):
            self.node.get_logger().info(
                "State 21: arm to DROP (straight-line cartesian descent)"
            )
            # Cartesian (straight-line) descent from pre-drop to drop, same as
            # the grasp descent. Pre-drop and drop share the orientation, so a
            # joint-space plan was free to spin the wrist (arm_7) a full 2*pi
            # and back -- the "360 deg wrist rotation before release". A
            # cartesian path holds the orientation fixed and just lowers the
            # gripper straight down.
            self.arm.move_to_pose(
                position=self._drop_pose_pos,
                quat_xyzw=self._drop_pose_quat,
                cartesian=True,
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

    def state_22_open_gripper_release(self, gripper_timeout):
        self.node.get_logger().info("State 22: opening gripper to release cube")
        self.gripper.open()
        # Visible delay so the gripper opening renders before detach.
        time.sleep(1.0)
        self.state = 23
        self._send_time = time.time()

    def state_23_detach(self, attach_timeout):
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

    def state_24_arm_lift_after_drop(self, arm_timeout):
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

    def state_25_arm_carry_back(self, arm_timeout):
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
            # The place push (State 27, "place" mode) parked us close to the
            # place wall, inside its inflation_radius. back_out (State 28,
            # "place" mode) reverses to SAFE_NAV_DISTANCE so the next-cube
            # Nav2 leg can plan, then -> State 26 (next cube or done).
            self._push_mode = "place"
            self._send_time = time.time()
            self.state = 28
            return
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn("Arm tuck-back timed out, retrying")
            self._clear_arm_goal_sent()
            return

    def state_26_next_or_done(self):
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
        # Re-open the tracker (it was frozen for the previous cube's grasp)
        # so the next cube's detections are accepted again.
        self.cube_tracker.unfreeze()
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
    def _cmd_vel_push(self, target_x, target_y, stop_dist, timeout,
                      steer, tag):
        # Shared closed-loop /cmd_vel push engine used by BOTH the pick
        # approach (State 27, steer=True toward the cube marker) and the
        # place approach (State 29, steer=False toward a fixed point ahead).
        # Reads the map->base_link TF each tick, drives forward at
        # DRIVE_SPEED until base_link is within `stop_dist` of
        # (target_x, target_y), optionally steering to null the heading
        # error. Returns "done", "timeout", or None (still pushing) so the
        # caller decides the next state.
        YAW_GAIN = 1.5
        YAW_MAX = 0.5
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            self.node.get_logger().warn(
                f"{tag}: TF lookup failed ({e}); stopping",
                throttle_duration_sec=2.0,
            )
            self.nav.stop()
            return None

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        dist = math.hypot(target_x - rx, target_y - ry)

        if dist <= stop_dist:
            self.nav.stop()
            # Final heading error to the target (how frontal we ended up).
            fqz = tf.transform.rotation.z
            fqw = tf.transform.rotation.w
            final_yaw = math.atan2(2.0 * fqw * fqz, 1.0 - 2.0 * fqz * fqz)
            final_bearing = math.atan2(target_y - ry, target_x - rx)
            final_err = math.atan2(math.sin(final_bearing - final_yaw),
                                   math.cos(final_bearing - final_yaw))
            self.node.get_logger().info(
                f"{tag} done: {dist:.2f} m from target "
                f"(stop {stop_dist:.2f} m), final yaw_err={final_err:.2f} rad"
            )
            return "done"

        if (time.time() - self._send_time) > timeout:
            self.nav.stop()
            self.node.get_logger().warn(
                f"{tag} timed out at {dist:.2f} m"
            )
            return "timeout"

        angular = 0.0
        yaw_err = 0.0
        if steer:
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w
            robot_yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
            bearing = math.atan2(target_y - ry, target_x - rx)
            yaw_err = math.atan2(math.sin(bearing - robot_yaw),
                                 math.cos(bearing - robot_yaw))
            angular = max(-YAW_MAX, min(YAW_MAX, YAW_GAIN * yaw_err))

        self.nav.publish_forward(DRIVE_SPEED, angular=angular)
        self.node.get_logger().info(
            f"{tag}: {dist:.2f} m -> stop {stop_dist:.2f} m, "
            f"yaw_err={yaw_err:.2f} rad",
            throttle_duration_sec=1.0,
        )
        return None

    def _push_reference_xy(self):
        # Returns the (x, y) map-frame reference point that State 27 / 28
        # measure distance against, selected by self._push_mode:
        #   "pick"  -> the cube marker (cube_tracker.cube_marker_in_map)
        #   "place" -> the place wall marker (aruco.place_marker_in_map)
        # Returns None if the relevant marker isn't available.
        if self._push_mode == "place":
            marker = self.aruco.place_marker_in_map
            if marker is None:
                return None
            return marker.p.x(), marker.p.y()
        marker = self.cube_tracker.cube_marker_in_map[self.current_cube_id]
        if marker is None:
            return None
        return marker.p.x(), marker.p.y()


    def _precompute_grasp_poses(self, marker_in_map):
        # Lab 3 style: build approach frame directly in base_link using the
        # marker seen by the camera (anti-drift), falling back to map frame.
        # DoRotY(+pi/2) makes X_gripper point straight down for top-down grasp.

        # --- primary: camera-frame marker transformed to base_link ----------
        marker_in_base = None
        marker_in_cam = self.cube_tracker.cube_marker_in_camera.get(
            self.current_cube_id
        )
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

        # --- fallback: map marker transformed to base_link ------------------
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

        # --- position -------------------------------------------------------
        cube_x = marker_in_base.p.x()
        cube_y = marker_in_base.p.y()
        # marker frame sits on TOP face; cube center is half-side below
        cube_center_z = marker_in_base.p.z() - CUBE_TOP_TO_CENTER
        cube_top_z = marker_in_base.p.z()          # = cube_center_z + CUBE_TOP_TO_CENTER

        # --- orientation (Lab 3 approach_frame modifier) --------------------
        # The ArUco PnP roll/pitch is noisy; using the full marker rotation
        # would tilt the approach axis and the gripper would NOT descend
        # vertically. Keep ONLY the marker yaw about base_link Z (this aligns
        # the fingers with the cube faces), then build the orientation from
        # Identity so the descent axis is guaranteed perfectly vertical:
        #   Identity -> DoRotZ(yaw): pure rotation about vertical, X/Y stay
        #               horizontal, fingers parallel to the cube faces.
        #   DoRotY(+pi/2): X_gripper now points along -Z_base (straight down).
        mqx, mqy, mqz, mqw = marker_in_base.M.GetQuaternion()
        cube_yaw = math.atan2(
            2.0 * (mqw * mqz + mqx * mqy),
            1.0 - 2.0 * (mqy * mqy + mqz * mqz),
        )
        grasp_orient = Rotation.Identity()
        grasp_orient.DoRotZ(cube_yaw)
        grasp_orient.DoRotY(math.pi / 2.0)

        grasp_in_base = Frame(
            grasp_orient,
            Vector(cube_x, cube_y, cube_top_z + GRASP_Z_ABOVE_TOP),
        )
        pre_grasp_in_base = Frame(
            grasp_orient,
            Vector(cube_x, cube_y, cube_center_z + PRE_GRASP_LIFT),
        )

        self._grasp_pose_pos, self._grasp_pose_quat = self._frame_to_pos_quat(
            grasp_in_base
        )
        self._pre_grasp_pose_pos, self._pre_grasp_pose_quat = self._frame_to_pos_quat(
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
        # We don't use a cube marker for the drop -- the cube IS in the
        # gripper. The target is "PLACE_FORWARD_OFFSET in front of the robot,
        # on the place surface". We compute it from the robot's CURRENT base
        # pose, NOT the latched place_approach_pose: the place push (State 27,
        # "place" mode) drives the base ~0.4 m closer to the wall AFTER the
        # approach pose was latched, so a drop measured from the stale
        # approach pose ended up only ~0.24 m in front of the base -- too
        # close for the arm to reach (OMPL "Unable to sample valid states").
        # Measuring from the current base keeps the drop a fixed, reachable
        # distance in front of the robot regardless of how far it pushed.
        tf_map_base = self.tf_buffer.lookup_transform(
            "map", "base_link",
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        rx = tf_map_base.transform.translation.x
        ry = tf_map_base.transform.translation.y
        bqz = tf_map_base.transform.rotation.z
        bqw = tf_map_base.transform.rotation.w
        robot_yaw_map = math.atan2(2.0 * bqw * bqz, 1.0 - 2.0 * bqz * bqz)

        # Drop XY = PLACE_FORWARD_OFFSET in front of the base along its yaw
        # (the robot faces the place wall after the steered place push).
        drop_x = rx + PLACE_FORWARD_OFFSET * math.cos(robot_yaw_map)
        drop_y = ry + PLACE_FORWARD_OFFSET * math.sin(robot_yaw_map) + (PLACE_LATERAL_OFFSET if self._current_cube_idx == 0 else -PLACE_LATERAL_OFFSET)

        # Posizione in base_link: PLACE_FORWARD_OFFSET m avanti lungo X_base,
        # offset laterale per separare i due cubi.
        drop_x_base = PLACE_FORWARD_OFFSET
        # Laterale ancorato al CENTRO TAVOLO (marker di place), non alla base:
        # proietto l'offset del marker sull'asse Y_base del robot.
        mk = self.aruco.place_marker_in_map
        table_y_base = (-(mk.p.x() - rx) * math.sin(robot_yaw_map)
                        + (mk.p.y() - ry) * math.cos(robot_yaw_map)) if mk else 0.0
        drop_y_base = table_y_base + (PLACE_LATERAL_OFFSET if self._current_cube_idx == 0 else -PLACE_LATERAL_OFFSET)

        # Altezza in base_link: convertiamo PLACE_TARGET_Z (quota mappa) in z_base.
        tf_base_map = self.tf_buffer.lookup_transform(
            "base_link", "map",
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        frame_base_map = transform_to_kdl_frame(tf_base_map)
        drop_in_map_for_z = frame_base_map * Frame(
            Rotation.Identity(), Vector(drop_x, drop_y, PLACE_TARGET_Z)
        )
        drop_z_base = drop_in_map_for_z.p.z()
        pre_drop_z_base = drop_z_base + POST_GRASP_LIFT

        # Orientamento: stesso pattern del grasp (DoRotY(-pi/2) = discesa verticale).
        drop_in_base = Frame(Rotation.Identity(), Vector(drop_x_base, drop_y_base, drop_z_base))
        drop_in_base.M.DoRotY(math.pi / 2.0)

        pre_drop_in_base = Frame(Rotation.Identity(), Vector(drop_x_base, drop_y_base, pre_drop_z_base))
        pre_drop_in_base.M.DoRotY(math.pi / 2.0)

        self._drop_pose_pos, self._drop_pose_quat = self._frame_to_pos_quat(
            drop_in_base
        )
        self._pre_drop_pose_pos, self._pre_drop_pose_quat = (
            self._frame_to_pos_quat(pre_drop_in_base)
        )
        self.node.get_logger().info(
            f"[diag] drop_map=({drop_x:.2f}, {drop_y:.2f}, {PLACE_TARGET_Z:.2f}) "
            f"drop_base=({self._drop_pose_pos[0]:.2f}, "
            f"{self._drop_pose_pos[1]:.2f}, {self._drop_pose_pos[2]:.2f})"
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
