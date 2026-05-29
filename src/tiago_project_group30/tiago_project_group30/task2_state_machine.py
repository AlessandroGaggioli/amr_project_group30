# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# Non-blocking state machine (modeled on Lab 4's state_machine_not_blocking).
# Fulfills the Task 2 specification:
#   State 0: tuck arm to HOME via MoveIt (FIRST: spawn pose has the arm
#            extended, which is dangerous near obstacles and blocks both
#            the LiDAR/depth FOV and ArUco detection).
#   State 1: wait for arm motion to finish.
#   State 2: AMCL global localization. Substates:
#              0  call /reinitialize_global_localization
#              1  clear local costmap (removes stale marks that would
#                 falsely trip "Collision Ahead" on Spin)
#              2  send Nav2 /spin 2*pi
#              3  wait for AMCL covariance to converge
#   State 3: random search by sampling reachable cells from the global
#            costmap, with visited memory and a per-waypoint /spin 2*pi
#            for the in-place sweep. Exit when both approach poses are
#            latched by the ArUco tracker.
#   State 4: navigate to PICK approach.
#   State 5: navigate to PLACE approach.
#   State 6: done.

import math
import time
from threading import Event

import rclpy


class StateMachine:
    def __init__(self, node, arm, nav, amcl, aruco, sampler):
        self.node = node
        self.arm = arm
        self.nav = nav
        self.amcl = amcl
        self.aruco = aruco
        self.sampler = sampler

        self.state = 0
        self.finished = False
        self.search_phase = "sample"

        # Last send-time used by per-state timeout watchers. The original
        # monolithic state machine kept this as a local variable; here we
        # promote it to an attribute so the per-state methods can share it.
        self._send_time = None

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self, executor_ready: Event):
        self.node.get_logger().info("State machine: waiting for executor...")
        executor_ready.wait()
        self.node.get_logger().info("Executor ready, giving the stack 10s to come up")
        time.sleep(10.0)

        # Per-goal timeouts (see comments inline below).
        PLANNING_TIMEOUT = 6.0
        SEARCH_NAV_TIMEOUT = 25.0
        # APPROACH_NAV_TIMEOUT raised from 60 -> 180 s: PICK -> PLACE
        # traversal can be ~5 m through tight pockets. When DWB cannot
        # make progress, Nav2's BT navigate_w_replanning_and_recovery
        # runs a recovery cycle (ClearLocalCostmap + Spin + Wait +
        # replan), each cycle ~10-15 s. At 60 s we were cancelling after
        # only 2-3 cycles and re-sending the same goal, restarting the
        # BT from scratch and losing all progress. 180 s lets the BT
        # exhaust ~10 recovery cycles before we give up.
        APPROACH_NAV_TIMEOUT = 180.0
        # /spin 2*pi at max_rotational_vel=1.0 rad/s nominally takes
        # ~6.3 s. 15 s leaves margin for Gazebo RTF + behavior_server.
        SEARCH_SPIN_TIMEOUT = 15.0

        while rclpy.ok() and not self.finished:
            try:
                self.arm.update_flags()
                self.nav.update_flags()
                self.amcl.update_spin_flags()
                self.amcl.update_amcl_flags()

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
                        # The State 3 helper returns True when it issued
                        # a `continue` equivalent (e.g. timeout watcher
                        # fired). Skip the trailing sleep so we react
                        # fast to the cancel/result update.
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

            except Exception as e:
                import traceback
                self.node.get_logger().error(
                    f"Exception in state machine: {e}\n{traceback.format_exc()}"
                )

            time.sleep(0.05)

    # ------------------------------------------------------------------
    # State 0: tuck arm to HOME
    # ------------------------------------------------------------------
    def _tick_state_0_arm_tuck(self):
        self.node.get_logger().info("State 0: tuck arm to HOME (MoveIt)")
        self.arm.move_to_home(tilt_head=True)
        self._send_time = time.time()
        self.state = 1

    # ------------------------------------------------------------------
    # State 1: wait for arm motion to finish
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # State 2: AMCL global localization
    # ------------------------------------------------------------------
    def _tick_state_2_amcl_localization(self):
        amcl = self.amcl
        if amcl.loc_substate == 0:
            self.node.get_logger().info(
                "State 2a: requesting AMCL global localization"
            )
            if amcl.request_global_localization():
                # Give AMCL a moment to redistribute particles before any
                # further action.
                time.sleep(2.0)
                amcl.loc_substate = 1
            else:
                time.sleep(2.0)  # service unavailable -> retry
        elif amcl.loc_substate == 1:
            self.node.get_logger().info(
                "State 2b: clearing local costmap before spin"
            )
            amcl.clear_local_costmap()
            time.sleep(0.5)  # let the costmap process the clear
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
            # Convergence wins over spin completion: as soon as AMCL is
            # confident we can move on, even mid-spin.
            if amcl.converged:
                self.node.get_logger().info(
                    "State 2d: AMCL aligned -- proceeding to search"
                )
                if amcl.spin_active and amcl.spin_goal_handle is not None:
                    amcl.cancel_spin()
                # Note: clear_global_costmap() is fired automatically
                # inside update_amcl_flags() on the convergence transition,
                # so it's already been called by this point.
                self.state = 3
                return
            if amcl.spin_done:
                # Spin finished but AMCL not yet converged.
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
                amcl.loc_substate = 1  # back to clear costmap

    # ------------------------------------------------------------------
    # State 3: random search w/ visited memory + per-waypoint spin
    # ------------------------------------------------------------------
    def _tick_state_3_random_search(self, search_nav_timeout, search_spin_timeout):
        # Returns True if a `continue` should be emitted (timeout watcher
        # fired or waiting for costmap), False otherwise.
        amcl = self.amcl
        nav = self.nav
        aruco = self.aruco

        # Exit condition: both approach poses latched.
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
            # Clear ALL goal state so a stale callback from the
            # cancelled in-flight goal doesn't bleed into State 4.
            nav.goal_active = False
            nav.goal_done = False
            nav.goal_succeeded = False
            amcl.spin_active = False
            amcl.spin_done = False
            amcl.spin_succeeded = False
            self.search_phase = "sample"
            # Freeze the approach poses now so Nav2's target doesn't
            # keep drifting during the final approach.
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

        # 1) Handle nav goal completion (only relevant in 'nav')
        if self.search_phase == "nav" and nav.goal_done:
            nav_ok = nav.goal_succeeded
            nav.goal_done = False
            if nav_ok:
                # Arrived at waypoint -> launch in-place Spin 2*pi
                self.node.get_logger().info(
                    "Waypoint reached, sending Spin 2*pi"
                )
                amcl.send_spin(2.0 * math.pi)
                self.search_phase = "spin"
                self._send_time = time.time()
            else:
                # Nav failed -> abandon this waypoint
                self.node.get_logger().warn(
                    "Waypoint nav failed -> sampling next"
                )
                self.search_phase = "sample"

        # 2) Handle spin completion (only relevant in 'spin')
        if self.search_phase == "spin" and amcl.spin_done:
            spin_ok = amcl.spin_succeeded
            amcl.spin_done = False
            if not spin_ok:
                self.node.get_logger().warn(
                    "Waypoint spin aborted -> sampling next"
                )
            # Either way: done with this waypoint, go sample next
            self.search_phase = "sample"

        # 3) Watch nav timeout
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

        # 4) Watch spin timeout
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

        # 5) Idle ('sample') -> draw a new waypoint and send nav
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
                # sample_random_xy returns None for either
                # (a) the visited set covers all reachable cells
                #     -> real saturation, clearing helps;
                # (b) costmap / TF lookup was transiently unavailable
                #     -> clearing visited does nothing useful.
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

    # ------------------------------------------------------------------
    # State 4: navigate to PICK approach
    # ------------------------------------------------------------------
    def _tick_state_4_pick(self, approach_nav_timeout):
        nav = self.nav
        # 1) Handle goal completion FIRST.
        if nav.goal_done:
            nav.goal_done = False  # consume
            if nav.goal_succeeded:
                self.node.get_logger().info(
                    "PICK approach reached, advancing to PLACE"
                )
                self.state = 5
                return True
            else:
                self.node.get_logger().warn(
                    "PICK nav did not succeed, retrying"
                )
                # fall through -> the (not active) branch below will
                # re-send the goal.

        # 2) In flight -> only watch for timeout.
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

        # 3) Not active -> send (initial or retry).
        x, y, yaw = self.nav.approach_pose_to_xy_yaw(
            self.aruco.pick_approach_pose
        )
        self.node.get_logger().info("State 4: nav to PICK approach")
        nav.send_goal(x, y, yaw)
        self._send_time = time.time()
        return False

    # ------------------------------------------------------------------
    # State 5: navigate to PLACE approach
    # ------------------------------------------------------------------
    def _tick_state_5_place(self, approach_nav_timeout):
        nav = self.nav
        # 1) Handle goal completion FIRST.
        if nav.goal_done:
            nav.goal_done = False  # consume
            if nav.goal_succeeded:
                self.node.get_logger().info(
                    "PLACE approach reached, task done"
                )
                self.state = 6
                return True
            else:
                self.node.get_logger().warn(
                    "PLACE nav did not succeed, retrying"
                )

        # 2) In flight -> only watch for timeout.
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

        # 3) Not active -> send (initial or retry).
        x, y, yaw = self.nav.approach_pose_to_xy_yaw(
            self.aruco.place_approach_pose
        )
        self.node.get_logger().info("State 5: nav to PLACE approach")
        nav.send_goal(x, y, yaw)
        self._send_time = time.time()
        return False

    # ------------------------------------------------------------------
    # State 6: done
    # ------------------------------------------------------------------
    def _tick_state_6_done(self):
        self.node.get_logger().info("State 6: Task 2 completed")
        self.finished = True
