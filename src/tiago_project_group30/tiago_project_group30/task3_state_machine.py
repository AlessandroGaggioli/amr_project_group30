# TASK 3 of Autonomous Mobile Robotics Exam - Group 30
#
# Self-contained state machine that performs Task 2 (random search +
# AMCL alignment + navigation to PICK / PLACE) AND the Task 3 pick-and-
# place sub-flow for two cubes (ID 63 first, ID 582 second).
#
# Phases are named (see the Phase IntEnum) rather than magic integers. The
# manipulation is data-driven: the GRASP and DROP flows are LISTS of steps
# executed by one Phase.RUN_SEQUENCE driver (_run_sequence), instead of a
# long ladder of near-identical numbered states. Each step is a small
# callable that drives a shared step-runner (StateRunners._arm_motion_step /
# _gripper_step / _link_step) and returns True when complete.
#
# Flow:
#   ARM_HOME -> WAIT_ARM -> AMCL -> SEARCH -> NAV_PICK
#   NAV_PICK reached -> HEAD_TILT -> WAIT_CUBE
#       (first detection) -> WAIT_CUBE_NAV -> PUSH(pick) -> WAIT_CUBE
#       (close detection) -> RUN_SEQUENCE(grasp) -> BACK_OUT(pick) -> NAV_PLACE
#   NAV_PLACE reached -> REFRESH_PLACE -> PUSH(place)
#       -> RUN_SEQUENCE(drop) -> BACK_OUT(place) -> NEXT_OR_DONE
#   NEXT_OR_DONE -> NAV_PICK (next cube) or DONE.

import math
import time
from enum import IntEnum
from threading import Event

import rclpy
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
    CUBE_APPROACH_DISTANCE,
    SAFE_NAV_DISTANCE,
    CUBE_PICK_SEQUENCE,
    CUBE_TOP_TO_CENTER,
    DRIVE_SPEED,
    GRASP_Z_ABOVE_TOP,
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


class Phase(IntEnum):
    # Shared phases -- values MATCH the integers CommonStates sets directly
    # (state_0..state_6), so IntEnum equality bridges the two.
    ARM_HOME = 0
    WAIT_ARM = 1
    AMCL = 2
    SEARCH = 3
    NAV_PICK = 4
    NAV_PLACE = 5
    DONE = 6
    # Task 3 phases.
    HEAD_TILT = 10
    WAIT_CUBE = 11
    WAIT_CUBE_NAV = 12
    RUN_SEQUENCE = 13     # executes the grasp OR the drop step list
    PUSH = 14             # closed-loop cmd_vel push (pick or place)
    BACK_OUT = 15         # straight reverse (pick or place)
    REFRESH_PLACE = 16
    NEXT_OR_DONE = 17


class Task3StateMachine(CommonStates):
    # Per-goal timeouts for the manipulation steps (used by the grasp/drop
    # sequences). The nav timeouts stay local to run().
    ARM_MOTION_TIMEOUT = 20.0   # MoveIt2 plan+execute per arm goal
    GRIPPER_TIMEOUT = 5.0
    ATTACH_TIMEOUT = 3.0

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

        self.state = Phase.ARM_HOME
        self.finished = False
        self.search_phase = "sample"
        self._send_time = None

        # Task 3 progress tracking.
        self._current_cube_idx = 0   # index into CUBE_PICK_SEQUENCE
        # One-shot latch: True after we have navigated to the cube
        # approach pose (so re-entering WAIT_CUBE with a fresh marker
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
        # used by the shared PUSH and BACK_OUT phases.
        self._push_mode = "pick"

        # Step-sequence machinery (grasp & drop are data-driven step lists
        # run by _run_sequence while self.state == Phase.RUN_SEQUENCE).
        self._seq = []
        self._seq_idx = 0
        self._seq_on_complete = None
        self._step_sent = False

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
        CUBE_DETECTION_TIMEOUT = 15.0   # max wait for cube marker after head tilt

        while rclpy.ok() and not self.finished:
            try:
                self.arm.update_flags()
                self.nav.update_flags()
                self.amcl.update_spin_flags()
                self.amcl.update_amcl_flags()
                self.gripper.update_flags()

                phase = self.state

                # ---- shared Task 2 phases (states 0-6 in CommonStates) ----
                if phase == Phase.ARM_HOME:
                    self.state_0_arm_tuck()
                elif phase == Phase.WAIT_ARM:
                    self.state_1_wait_arm(PLANNING_TIMEOUT, self.ARM_MOTION_TIMEOUT)
                elif phase == Phase.AMCL:
                    self.state_2_amcl_localization()
                elif phase == Phase.SEARCH:
                    if self.state_3_random_search(SEARCH_NAV_TIMEOUT):
                        continue
                elif phase == Phase.NAV_PICK:
                    if self.state_4_pick(APPROACH_NAV_TIMEOUT):
                        continue
                elif phase == Phase.NAV_PLACE:
                    if self.state_5_place(APPROACH_NAV_TIMEOUT):
                        continue
                elif phase == Phase.DONE:
                    self.state_6_done()
                    break

                # ---- Task 3 phases ----
                elif phase == Phase.HEAD_TILT:
                    self._phase_head_tilt()
                elif phase == Phase.WAIT_CUBE:
                    self._phase_wait_cube(CUBE_DETECTION_TIMEOUT)
                elif phase == Phase.WAIT_CUBE_NAV:
                    self._phase_wait_cube_nav(APPROACH_NAV_TIMEOUT)
                elif phase == Phase.RUN_SEQUENCE:
                    self._run_sequence()
                elif phase == Phase.PUSH:
                    self._phase_push()
                elif phase == Phase.BACK_OUT:
                    self._phase_back_out()
                elif phase == Phase.REFRESH_PLACE:
                    self._phase_refresh_place(CUBE_DETECTION_TIMEOUT)
                elif phase == Phase.NEXT_OR_DONE:
                    self._phase_next_or_done()

            except Exception as e:
                import traceback
                self.node.get_logger().error(
                    f"Exception in state machine: {e}\n{traceback.format_exc()}"
                )

            time.sleep(0.05)

    # ==================================================================
    # Task 2 nav overrides (advance into the Task 3 sub-flow on success)
    # ==================================================================
    def state_4_pick(self, approach_nav_timeout):
        # CHANGE vs. Task 2: on success advance to HEAD_TILT (cube pick flow).
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
        # CHANGE vs. Task 2: on success advance to REFRESH_PLACE (re-detect the
        # far-frozen place marker up close, then push + drop).
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

    # ==================================================================
    # Task 3 cube-approach phases
    # ==================================================================
    def _phase_head_tilt(self):
        # Enable the CubeTracker only NOW: during Task 2 random search the
        # camera could glimpse a cube from far/oblique angle and store a noisy
        # PnP that WAIT_CUBE would reuse before the head finishes tilting.
        self.cube_tracker.enable()
        # Single tilt straight down, no pan. The wall-marker approach already
        # aligns the robot with the surface, so the cube should be in front;
        # the strict TF-sync filter in the cube tracker drops noisy detections
        # during head motion and accepts the first synced one once it settles.
        self.node.get_logger().info(
            f"HEAD_TILT: tilt={HEAD_TILT_DOWN_FOR_CUBE} rad, pan=0.0 rad"
        )
        self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, 0.0)
        self._send_time = time.time()
        self._head_scan_active = False
        self._head_scan_idx = 0
        self._head_scan_time = None
        self.state = Phase.WAIT_CUBE

    def _phase_wait_cube(self, detection_timeout):
        cube_id = self.current_cube_id
        marker = self.cube_tracker.cube_marker_in_map[cube_id]
        if marker is not None:
            self.node.get_logger().info(
                f"Cube ID {cube_id} marker localized at map "
                f"({marker.p.x():.2f}, {marker.p.y():.2f}, {marker.p.z():.2f})"
            )
            # First detection: the wall-marker approach pose typically leaves
            # the cube outside the arm workspace. Navigate to a cube-centric
            # approach pose before attempting the grasp.
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
            # Second pass (after Nav2 + push placed us in front of the cube):
            # closest-observation policy means cube_marker_in_map is now the
            # refined short-range estimate. FREEZE it so a late noisy detection
            # can't shift the target sideways during the grasp, compute the
            # grasp poses, then start the grasp sequence.
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
        # Wait-only on the goal already issued by _send_cube_approach_nav.
        # Shares the result-consume + timeout-guard with states 4/5, but the
        # success/failure branches are bespoke (no resend).
        res = self._consume_nav_result()
        if res is True:
            self.node.get_logger().info(
                "Cube approach Nav2 reached (within inflation limit); "
                "closing remaining gap with a /cmd_vel push"
            )
            # Re-tilt head straight down so the closer detection is captured
            # while we push the base forward.
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

    # ==================================================================
    # PUSH: closed-loop /cmd_vel push toward a reference frame, steering
    # frontal, used for BOTH pick and place (selected by self._push_mode).
    # Nav2's inflation_radius parks the base too far from the target; this
    # drives the remaining gap while nulling the heading error.
    #   pick  -> toward the cube marker, stop within CUBE_APPROACH_DISTANCE,
    #            then re-detect close & grasp (WAIT_CUBE).
    #   place -> toward the place wall marker, stop within
    #            PLACE_APPROACH_DISTANCE, then drop (RUN_SEQUENCE).
    # ==================================================================
    def _phase_push(self):
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

        stop_dist = CUBE_APPROACH_DISTANCE if is_pick else PLACE_APPROACH_DISTANCE
        result = self._cmd_vel_push(
            ref[0], ref[1], stop_dist=stop_dist,
            timeout=40.0, steer=True, tag=f"push ({self._push_mode})",
        )

        if result is not None:
            if is_pick:
                self._cube_approached = True
                if result == "done" or result == "timeout":
                    # Tilt the head a bit further down to centre the marker up
                    # close for a clean optical read.
                    self.arm.tilt_head(-1.15, 0.0)
                    # Pause to let the base inertia settle.
                    time.sleep(1.5)
                    # Drop the data recorded while moving so the close-range
                    # detection wins.
                    self.cube_tracker.reset_cube(self.current_cube_id)
                    self.cube_tracker.unfreeze()
                self.state = Phase.WAIT_CUBE
            else:
                self._start_drop_sequence()

    def _phase_back_out(self):
        # Straight reverse, the mirror of PUSH. Reverses away from the
        # reference frame until base_link is at least SAFE_NAV_DISTANCE from
        # it, clearing the surface's inflation_radius so Nav2 can plan again.
        #   pick  -> after grasp/tuck, then -> NAV_PLACE.
        #   place -> after drop/tuck, then -> NEXT_OR_DONE.
        BACK_TIMEOUT = 60.0
        next_state = Phase.NAV_PLACE if self._push_mode != "place" \
            else Phase.NEXT_OR_DONE

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

        # Far enough from the surface -> stop and go to the next leg.
        if dist >= (SAFE_NAV_DISTANCE+0.5):
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

        # Still inside the zone -> keep reversing (negative speed).
        self.nav.publish_forward(-DRIVE_SPEED)
        self.node.get_logger().info(
            f"back_out ({self._push_mode}): {dist:.2f} m -> target "
            f"{SAFE_NAV_DISTANCE:.2f} m",
            throttle_duration_sec=1.0,
        )

    def _phase_refresh_place(self, timeout):
        # The place marker was localized from far away during the search and
        # then frozen, so the push and the drop both targeted a wrong point.
        # Now that we're parked in front of the surface, tilt the head down to
        # the (low) marker and let a FRESH close-range detection overwrite
        # place_marker_in_map before the push (same anti-drift as the cube).
        if not self._arm_goal_sent_for_state(Phase.REFRESH_PLACE):
            self.node.get_logger().info(
                "REFRESH_PLACE: re-detecting place marker up close before drop"
            )
            # Un-freeze + reset KEEP-CLOSEST so the close detection wins over
            # the stale far-away one.
            self.aruco.place_detection_distance = float("inf")
            self.aruco._refresh_approach_poses = True
            # Start a head scan (tilt down + pan sweep) to bring the low place
            # marker into the FOV, same as the pick cube search: the robot
            # rarely parks dead-centre on the surface, so a fixed pan=0 can
            # miss the marker.
            self._head_scan_idx = 0
            self._head_scan_time = time.time()
            self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, HEAD_SCAN_PANS[0])
            self._mark_arm_goal_sent(Phase.REFRESH_PLACE)
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
            self.state = Phase.PUSH
            return

        # Not refreshed yet -> keep sweeping the head across the pan positions.
        self._advance_head_scan("Place marker scan")

    def _phase_next_or_done(self):
        finished_cube_id = self.current_cube_id
        self.node.get_logger().info(
            f"NEXT_OR_DONE: cube ID {finished_cube_id} placed"
        )
        self._current_cube_idx += 1
        if self._current_cube_idx >= len(CUBE_PICK_SEQUENCE):
            self.node.get_logger().info(
                "All cubes placed -> Task 3 done"
            )
            self.state = Phase.DONE
            return
        # Next cube: clean nav state, reset the cube tracker entry for the
        # upcoming marker (in case a stale long-range observation was kept
        # while we were busy with the previous cube), then go back to NAV_PICK.
        next_id = self.current_cube_id
        self.node.get_logger().info(
            f"Next cube to pick: ID {next_id} -> nav back to PICK"
        )
        self.cube_tracker.reset_cube(next_id)
        # Re-open the tracker (it was frozen for the previous cube's grasp)
        # so the next cube's detections are accepted again.
        self.cube_tracker.unfreeze()
        # Reset the cube-approach flag so the next cube also triggers its own
        # short-range nav refinement in WAIT_CUBE.
        self._cube_approached = False
        self.nav.reset_latches()
        self.state = Phase.NAV_PICK

    # ==================================================================
    # Step-sequence machinery (RUN_SEQUENCE driver + grasp/drop builders)
    # ==================================================================
    def _start_sequence(self, steps, on_complete):
        self._seq = steps
        self._seq_idx = 0
        self._step_sent = False
        self._seq_on_complete = on_complete
        self.state = Phase.RUN_SEQUENCE

    def _run_sequence(self):
        # Run the current step once; advance when it reports done. When the
        # list is exhausted, fire the on-complete branch (which moves to the
        # next phase).
        if self._seq_idx >= len(self._seq):
            return
        if self._seq[self._seq_idx]():
            self._seq_idx += 1
            self._step_sent = False
            if self._seq_idx >= len(self._seq):
                self._seq_on_complete()

    def _enter_back_out(self, mode):
        self._push_mode = mode
        self._send_time = time.time()
        self.state = Phase.BACK_OUT

    def _start_grasp_sequence(self):
        a = self.ARM_MOTION_TIMEOUT
        self._start_sequence([
            lambda: self._gripper_step(
                opening=True, log="GRASP: opening gripper before approach"),
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._pre_grasp_pose_pos,
                    quat_xyzw=self._pre_grasp_pose_quat),
                a, log="GRASP: arm to PRE-GRASP (above cube)",
                timeout_msg="Arm pre-grasp timed out, retrying"),
            # Cartesian descent so the gripper drops STRAIGHT DOWN onto the
            # cube (a joint-space plan curves in sideways and clips it).
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._grasp_pose_pos,
                    quat_xyzw=self._grasp_pose_quat, cartesian=True),
                a, log="GRASP: arm to GRASP (cartesian descent)",
                timeout_msg="Arm grasp timed out, retrying"),
            # Poll the close so the fingers settle before the rigid attach.
            lambda: self._gripper_step(
                opening=False, log="GRASP: closing gripper around cube",
                wait=True, gripper_timeout=self.GRIPPER_TIMEOUT, done_settle=0.5,
                timeout_msg="Gripper close timed out; attaching anyway"),
            lambda: self._link_step(
                self.link_attacher.attach, self.ATTACH_TIMEOUT,
                fail_msg="Attach link failed; retrying attach",
                timeout_msg="Attach service timed out, retrying",
                advance_on_timeout=False),
            # Cartesian lift (inverse of the descent) so the grasped cube
            # rises straight up without swinging into a neighbour/table edge.
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._pre_grasp_pose_pos,
                    quat_xyzw=self._pre_grasp_pose_quat, cartesian=True),
                a, log="GRASP: lifting arm (cartesian lift)",
                timeout_msg="Arm lift timed out, retrying",
                diag_fn=lambda: self._log_gripper_z("lift")),
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_home(tilt_head=False),
                a, log="GRASP: tucking arm to HOME for navigation",
                timeout_msg="Arm tuck timed out, retrying",
                diag_fn=lambda: self._log_gripper_z("tuck")),
        ], on_complete=self._grasp_complete)

    def _grasp_complete(self):
        self.node.get_logger().info(
            "Carry config ready -> backing out of pick zone"
        )
        # Raise the head back to level for the drive to PLACE.
        self.arm.tilt_head(0.0, 0.0)
        # The push parked us inside the pick surface's inflation_radius;
        # back out to SAFE_NAV_DISTANCE before the PLACE nav.
        self._enter_back_out("pick")

    def _start_drop_sequence(self):
        a = self.ARM_MOTION_TIMEOUT
        self._start_sequence([
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._pre_drop_pose_pos,
                    quat_xyzw=self._pre_drop_pose_quat),
                a, log="DROP: arm to PRE-DROP (above place surface)",
                timeout_msg="Arm pre-drop timed out, retrying",
                prepare=self._precompute_drop_poses,
                prepare_err="Failed to compute drop poses"),
            # Cartesian descent (holds orientation, no wrist spin-around).
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._drop_pose_pos,
                    quat_xyzw=self._drop_pose_quat, cartesian=True),
                a, log="DROP: arm to DROP (cartesian descent)",
                timeout_msg="Arm drop timed out, retrying"),
            lambda: self._gripper_step(
                opening=True, log="DROP: opening gripper to release cube"),
            lambda: self._link_step(
                self.link_attacher.detach, self.ATTACH_TIMEOUT,
                fail_msg="Detach link failed; continuing anyway",
                timeout_msg="Detach service timed out",
                advance_on_timeout=True),
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._pre_drop_pose_pos,
                    quat_xyzw=self._pre_drop_pose_quat),
                a, log="DROP: lifting arm clear of dropped cube",
                timeout_msg="Arm lift-after-drop timed out, retrying"),
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_home(tilt_head=False),
                a, log="DROP: tucking arm to HOME for nav back",
                timeout_msg="Arm tuck-back timed out, retrying"),
        ], on_complete=lambda: self._enter_back_out("place"))

    # ==================================================================
    # Helpers
    # ==================================================================
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
        wall_yaw = quat_to_yaw(
            pick_approach.pose.orientation.x,
            pick_approach.pose.orientation.y,
            pick_approach.pose.orientation.z,
            pick_approach.pose.orientation.w,
        )

        yaw = wall_yaw
        ax = cube_x - SAFE_NAV_DISTANCE * math.cos(yaw)
        ay = cube_y - SAFE_NAV_DISTANCE * math.sin(yaw)

        # Diagnostic: current robot->cube distance (just for the log,
        # not used in the math).
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
        # Shared closed-loop /cmd_vel push engine used by BOTH the pick
        # approach (PUSH pick, steer toward the cube marker) and the place
        # approach (PUSH place). Reads the map->base_link TF each tick, drives
        # forward at DRIVE_SPEED until base_link is within `stop_dist` of
        # (target_x, target_y), optionally steering to null the heading error.
        # Returns "done", "timeout", or None (still pushing).
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

        if (time.time() - self._send_time) > timeout:
            self.nav.stop()
            self.node.get_logger().warn(
                f"{tag} timed out at {dist:.2f} m"
            )
            return "timeout"

        angular = 0.0
        yaw_err = 0.0
        if steer:
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
        # Returns the (x, y) map-frame reference point that PUSH / BACK_OUT
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
        cube_yaw = quat_to_yaw(mqx, mqy, mqz, mqw)
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
        # We don't use a cube marker for the drop -- the cube IS in the
        # gripper. The target is "PLACE_FORWARD_OFFSET in front of the robot,
        # on the place surface". We compute it from the robot's CURRENT base
        # pose, NOT the latched place_approach_pose: the place push drives the
        # base ~0.4 m closer to the wall AFTER the approach pose was latched,
        # so a drop measured from the stale approach pose ended up only
        # ~0.24 m in front of the base -- too close for the arm to reach (OMPL
        # "Unable to sample valid states"). Measuring from the current base
        # keeps the drop a fixed, reachable distance in front of the robot.
        rx, ry, robot_yaw_map = self._robot_xy_yaw()

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
        # Su pavimento piano la X/Y del punto map non incide sulla z risultante,
        # quindi usiamo la posizione corrente del robot (rx, ry) come punto map.
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

        # Orientamento: stesso pattern del grasp (DoRotY(-pi/2) = discesa verticale).
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
