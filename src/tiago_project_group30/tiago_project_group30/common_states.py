# Autonomous Mobile Robotics Exam - Group 30

# Common state implementations shared by Task 2 and Task 3 state machines.

import math
import time
from enum import IntEnum

from tiago_project_group30.constants import (
    SEARCH_PAN_DWELL,
    SEARCH_PAN_POSITIONS,
)
from tiago_project_group30.state_helpers import StateRunners

class Phase(IntEnum):
    # Shared states 
    ARM_HOME = 0
    WAIT_ARM = 1
    AMCL = 2
    SEARCH = 3
    NAV_PICK = 4
    NAV_PLACE = 5
    DONE = 6
    # Task 3 only states 
    HEAD_TILT = 10
    WAIT_CUBE = 11
    WAIT_CUBE_NAV = 12
    RUN_SEQUENCE = 13     # executes the grasp OR the drop step list
    PUSH = 14             # closed-loop cmd_vel push (pick or place)
    BACK_OUT = 15         # straight reverse (pick or place)
    REFRESH_PLACE = 16
    NEXT_OR_DONE = 17


class CommonStates(StateRunners):
    # ------------------------------------------------------------------
    # State 0: tuck arm to home
    # ------------------------------------------------------------------
    def state_0_arm_tuck(self):
        self.node.get_logger().info("State 0: tuck arm to home")
        self.arm.move_to_home(tilt_head=False) 
        self._send_time = time.time()
        self.state = 1

    # ------------------------------------------------------------------
    # State 1: wait for arm motion to finish
    # ------------------------------------------------------------------
    def state_1_wait_arm(self, planning_timeout, execution_timeout):
        # Two retry guards, both required:
        #   - planning_timeout: the goal was sent but never started EXECUTING
        #     (planning failed / MoveIt not ready yet).
        #   - execution_timeout: the motion STARTED but stalled mid-execution
        #     (otherwise we would wait on motion_done forever).
        elapsed = time.time() - self._send_time
        if self.arm.motion_done:
            self.node.get_logger().info("Arm at HOME")
            self.state = 2
        elif not self.arm.motion_started and elapsed > planning_timeout:
            self.node.get_logger().warn("Arm planning timed out, retrying")
            self.state = 0
        elif self.arm.motion_started and elapsed > execution_timeout:
            self.node.get_logger().warn("Arm execution timed out, retrying")
            self.state = 0

    # ------------------------------------------------------------------
    # State 2: AMCL global localization
    # ------------------------------------------------------------------
    def state_2_amcl_localization(self):
        amcl = self.amcl

        #--- amcl localization substates ---

        # --- 2a: request global localization service ---
        if amcl.loc_substate == 0:
            self.node.get_logger().info(
                "State 2a: requesting AMCL global localization"
            )
            if amcl.request_global_localization():
                # Give AMCL time to redistribute particles before any action.
                time.sleep(2.0)
                amcl.loc_substate = 1
            else:
                time.sleep(2.0)  # service unavailable -> retry

        # --- 2b: clear local costmap ---
        elif amcl.loc_substate == 1:
            self.node.get_logger().info(
                "State 2b: clearing local costmap before spin"
            )
            amcl.clear_local_costmap()
            time.sleep(0.5)  # let the clear propagate before spinning
            amcl.loc_substate = 2

        # --- 2c: send spin goal ---
        elif amcl.loc_substate == 2:
            amcl.localization_spin_count += 1
            self.node.get_logger().info(
                f"State 2c: spin attempt "
                f"#{amcl.localization_spin_count} (target_yaw=2*pi)"
            )
            amcl.send_spin(2.0 * math.pi)
            self._send_time = time.time()
            amcl.loc_substate = 3

        # --- 2d: monitor spin + convergence ---
        elif amcl.loc_substate == 3:
            # Convergence wins over spin completion: 
            # as soon as AMCL is confident we can move on

            if amcl.converged:
                self.node.get_logger().info(
                    "State 2d: AMCL aligned -- proceeding to search"
                )
                if amcl.spin_active and amcl.spin_goal_handle is not None:
                    amcl.cancel_spin()
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
    # State 3: random search with visited memory + waypoint head pan
    # ------------------------------------------------------------------
    def state_3_random_search(self, search_nav_timeout):

        # Manages robot's exploration phase
            # If both markers (pick and place) are identified at any time
            # the function cancels the ongoing navigation
            # resets the navigator flags 
            # freezes the estimated poses (to prevent drift)
            # advance the system to state 4 

        # Search Phases: 
            # sample: generates a new random waypoint by reading the global costmap. 
                    # When a point is visited, it is added to the visited list. 
                    # If the sampling returns no points, the visited list is cleared to retry. 
            # nav: sends a navigation goal to the new waypoint and waits for completion or timeout
            # pan: when the robot reaches the waypoint, it starts a head pan sweep to look for the markers.
                    # Upon completing the full scan, if the markers have not yet been found
                    # the system goes back to the 'sample' phase to pick a new waypoint.

        nav = self.nav 
        aruco = self.aruco
        arm = self.arm

        # Exit condition: both markers have been spotted by the ArUco tracker.
        if (
            aruco.pick_approach_pose is not None
            and aruco.place_approach_pose is not None
        ):
            # if the robot is moving towards a waypoint, stop it. 
            if nav.goal_active:
                self.node.get_logger().info(
                    "Both markers seen mid-search, cancelling nav"
                )
                nav.cancel()


            # Reset all navigation tracking flags 
            nav.goal_active = False
            nav.goal_done = False
            nav.goal_succeeded = False

            # Reset the camera head to a neutral position, forward looking. 
            arm.tilt_head(-0.5, 0.0)

            # Reset internal search tracking variables 
            self.search_phase = "sample"
            self._pan_idx = 0

            # Lock the currently estimated coordinates of both markers. 
            # It prevents updates causing target drift while Nav2 plan and execute the approach. 
            aruco.freeze()
            self.node.get_logger().info(
                f"Freezing PICK approach at "
                f"({aruco.pick_approach_pose.pose.position.x:.2f}, "
                f"{aruco.pick_approach_pose.pose.position.y:.2f}) and "
                f"PLACE approach at "
                f"({aruco.place_approach_pose.pose.position.x:.2f}, "
                f"{aruco.place_approach_pose.pose.position.y:.2f})"
            )

            # Advance to State 4. 
            self.state = 4
            return True

        # Lazy-init the pan cursor on first entry.
        if not hasattr(self, "_pan_idx"):
            self._pan_idx = 0

        # Watch for nav completion (only in 'nav' phase)
        if self.search_phase == "nav" and nav.goal_done:

            # capture and reset nav result flags
            nav_ok = nav.goal_succeeded 
            nav.goal_done = False

            if nav_ok:
                # Arrived at waypoint , start head pan. 
                self._pan_idx = 0
                pan = SEARCH_PAN_POSITIONS[self._pan_idx]
                self.node.get_logger().info(
                    f"Waypoint reached, starting head pan sweep "
                    f"[{self._pan_idx + 1}/{len(SEARCH_PAN_POSITIONS)}] "
                    f"pan={pan:.2f} rad"
                )
                arm.tilt_head(-0.5, pan)

                # Transition to scanning phase. 
                self.search_phase = "pan"
                self._send_time = time.time()

            else:
                # Nav failed , abandon this waypoint
                self.node.get_logger().warn(
                    "Waypoint nav failed - sampling next"
                )
                self.search_phase = "sample"

        # Advance pan sweep when dwell time elapses
        if self.search_phase == "pan":

            # check if the camera ha dwelled at the current pan angle 
            if (time.time() - self._send_time) >= SEARCH_PAN_DWELL:
                self._pan_idx += 1

                if self._pan_idx >= len(SEARCH_PAN_POSITIONS):
                    # Sweep complete and markers not found. 
                    # Go back to sample phase to move to a new waypoint and try again.
                    self.search_phase = "sample"
                else:
                    # Move to the next pan position and reset the dwell timer.
                    pan = SEARCH_PAN_POSITIONS[self._pan_idx]
                    self.node.get_logger().info(
                        f"Head pan "
                        f"[{self._pan_idx + 1}/{len(SEARCH_PAN_POSITIONS)}] "
                        f"pan={pan:.2f} rad"
                    )
                    arm.tilt_head(-0.5, pan)
                    self._send_time = time.time()

        # Watch for nav timeout (only in 'nav' phase)
        if self.search_phase == "nav" and nav.goal_active:
            # If the robot has been navigating longer than the allowed threshold without success.
            if (
                not nav.timeout_fired
                and (time.time() - self._send_time) > search_nav_timeout
            ):
                self.node.get_logger().warn(
                    "Nav goal timed out, cancelling -> sample next"
                )
                # Cancel the active goal and flag the timeout. 
                nav.cancel()
                nav.timeout_fired = True
            return True

        # Handle waypoint generation during the sample phase. 
        if self.search_phase == "sample":
            # Ensure global_costmap is available vefore sampling.
            if self.sampler.costmap_msg is None:
                self.node.get_logger().info(
                    "Waiting for /global_costmap/costmap...",
                    throttle_duration_sec=2.0,
                )
                time.sleep(0.5)
                return True
            
            # Sample a random reachable waypoint from the global costmap.
            xy = self.sampler.sample_random_xy()


            if xy is None:
                # Sampling returns None if the robot has explored all reachable cells 
                # OR if the costmap lookup failed. 
                # Clear the visited history to reset the search space and retry. 
                self.node.get_logger().warn(
                    "Sample returned None, clearing visited "
                    "list and retrying",
                    throttle_duration_sec=2.0,
                )
                self.sampler.visited_waypoints = []
                time.sleep(0.5)
                return True
            
            # Sampled a valid waypoint, send the nav goal.
            x, y , yaw = xy
            # Append the new waypoint to the visited list to avoid re-sampling it in the future.
            self.sampler.visited_waypoints.append((x, y))
            self.node.get_logger().info(
                f"New random waypoint ({x:.2f}, {y:.2f}), {yaw:.2f};"
                f"visited={len(self.sampler.visited_waypoints)}"
            )

            # Send the navigation goal to the new waypoint and transition to nave phase. 
            nav.send_goal(x, y, yaw)
            self.search_phase = "nav"
            self._send_time = time.time()
        return False
    
    # # ------------------------------------------------------------------
    # # State 3: MANUAL search via RViz + autonomous head pan sweep -- 
    # # ACTIVATE TO FIND MARKERS W/O RANDOM EXPLORATION AND PROCEED TO APPROACH
    # # ------------------------------------------------------------------
    # def state_3_random_search(self, search_nav_timeout):
    #     aruco = self.aruco
    #     arm = self.arm

    #     # Exit condition: both markers have been spotted by the ArUco tracker.
    #     if (
    #         aruco.pick_approach_pose is not None
    #         and aruco.place_approach_pose is not None
    #     ):
    #         self.node.get_logger().info(
    #             "Entrambi i marker visti! Prendo il controllo e interrompo la navigazione RViz."
    #         )
            
    #         # Reset the camera head to a neutral position, forward looking. 
    #         arm.tilt_head(-0.5, 0.0)
            
    #         # Freeze the poses to prevent drift while Nav2 plans and executes the approach.
    #         aruco.freeze()
    #         self.node.get_logger().info(
    #             f"Freezing PICK approach at "
    #             f"({aruco.pick_approach_pose.pose.position.x:.2f}, "
    #             f"{aruco.pick_approach_pose.pose.position.y:.2f}) and "
    #             f"PLACE approach at "
    #             f"({aruco.place_approach_pose.pose.position.x:.2f}, "
    #             f"{aruco.place_approach_pose.pose.position.y:.2f})"
    #         )

    #         # Transition to State 4.
    #         self.state = 4
    #         return True

    #     # Manual search initialization - only executed on the first entry to State 3.
    #     if getattr(self, "_manual_search_init", None) is None:
    #         self.node.get_logger().info("Manual searcg active: set Nav2 Goal")
    #         self._manual_search_init = True
    #         self._pan_idx = 0
    #         self._send_time = time.time()
    #         arm.tilt_head(-0.5, SEARCH_PAN_POSITIONS[self._pan_idx])

    #     # Continuous head panning
    #     if (time.time() - self._send_time) >= SEARCH_PAN_DWELL:
    #         self._pan_idx = (self._pan_idx + 1) % len(SEARCH_PAN_POSITIONS)
    #         pan = SEARCH_PAN_POSITIONS[self._pan_idx]
    #         arm.tilt_head(-0.5, pan)
    #         self._send_time = time.time()

    #     return False

    # ------------------------------------------------------------------
    # State 4: navigate to PICK approach position
    # ------------------------------------------------------------------
    def state_4_pick(self, approach_nav_timeout):
        return self._nav_to_approach(
            self.aruco.pick_approach_pose,
            next_state=5,
            timeout=approach_nav_timeout,
            label="PICK",
            success_log="PICK approach reached, advancing to PLACE",
            send_log="State 4: nav to PICK approach",
        )

    # ------------------------------------------------------------------
    # State 5: navigate to PLACE approach
    # ------------------------------------------------------------------
    def state_5_place(self, approach_nav_timeout):
        return self._nav_to_approach(
            self.aruco.place_approach_pose,
            next_state=6,
            timeout=approach_nav_timeout,
            label="PLACE",
            success_log="PLACE approach reached, task done",
            send_log="State 5: nav to PLACE approach",
        )

    # ------------------------------------------------------------------
    # State 6: done
    # ------------------------------------------------------------------
    def state_6_done(self):
        # Final state. 
        self.node.get_logger().info("State 6: Task completed")
        self.finished = True
