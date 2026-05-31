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
#            costmap, with visited memory and a per-waypoint HEAD PAN
#            sweep for the in-place look-around (replaces the old Nav2
#            /spin 2*pi, which kept aborting on DWB critics). Exit when
#            both approach poses are latched by the ArUco tracker.
#   State 4: navigate to PICK approach.
#   State 5: navigate to PLACE approach.
#   State 6: done.

import math
import time
from threading import Event

import rclpy
from tiago_project_group30.common_states import CommonStates


class StateMachine(CommonStates):
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
        SEARCH_NAV_TIMEOUT = 120.0
        # APPROACH_NAV_TIMEOUT raised from 60 -> 180 s: PICK -> PLACE
        # traversal can be ~5 m through tight pockets. When DWB cannot
        # make progress, Nav2's BT navigate_w_replanning_and_recovery
        # runs a recovery cycle (ClearLocalCostmap + Spin + Wait +
        # replan), each cycle ~10-15 s. At 60 s we were cancelling after
        # only 2-3 cycles and re-sending the same goal, restarting the
        # BT from scratch and losing all progress. 180 s lets the BT
        # exhaust ~10 recovery cycles before we give up.
        APPROACH_NAV_TIMEOUT = 180.0

        while rclpy.ok() and not self.finished:
            try:
                self.arm.update_flags()
                self.nav.update_flags()
                self.amcl.update_spin_flags()
                self.amcl.update_amcl_flags()

                if self.state == 0:
                    self.state_0_arm_tuck()
                elif self.state == 1:
                    self.state_1_wait_arm(PLANNING_TIMEOUT)
                elif self.state == 2:
                    self.state_2_amcl_localization()
                elif self.state == 3:
                    if self.state_3_random_search(SEARCH_NAV_TIMEOUT):
                        # The State 3 helper returns True when it issued
                        # a `continue` equivalent (e.g. timeout watcher
                        # fired). Skip the trailing sleep so we react
                        # fast to the cancel/result update.
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

            except Exception as e:
                import traceback
                self.node.get_logger().error(
                    f"Exception in state machine: {e}\n{traceback.format_exc()}"
                )

            time.sleep(0.05)

    
