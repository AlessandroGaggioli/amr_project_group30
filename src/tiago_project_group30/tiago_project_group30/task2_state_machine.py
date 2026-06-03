# Autonomous Mobile Robotics Exam - Group 30
#
# Task 2 - Main State Machine 

# Workflow: 

    # State 0: Tuck the arm to a safe configuration for navigation.
    # State 1: Wait for the arm to finish moving. Timeout and retry on failure.
    # State 2: Localize with AMCL. 
        # substeps: scatter particles, clear costmap memory, spin, wait convergence. 
    # State 3: Explore the map randomly. 
        # substeps: sample random reachable pose, send nav goal, wait for result or timeout, repeat until both markers are detected.
    # State 4: Drive to the pick position 
    # State 5: Drive to the place position
    # State 6: Task completed.

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

        # track substates during random search (state 3). 
        self.search_phase = "sample"

        # Last send-time. Used for timeouts.
        self._send_time = None

    # -----------
    # Main loop
    # -----------
    def run(self, executor_ready: Event):
        # Wait for ROS2 system 
        self.node.get_logger().info("State machine: waiting for executor...")
        executor_ready.wait()
        self.node.get_logger().info("Executor ready, giving the stack 10s to come up")
        time.sleep(10.0)

        # timeouts
        PLANNING_TIMEOUT = 6.0 # Time allowed for MoveIt to plan an arm path
        EXECUTION_TIMEOUT = 30.0  # Time allowed for the physical arm movement
        SEARCH_NAV_TIMEOUT = 120.0 # Time allowed to reach a random exploration point
        APPROACH_NAV_TIMEOUT = 180.0 # Time allowed to reach the pick/place approach pose.

        # Main state machine loop
        while rclpy.ok() and not self.finished:
            try:
                # Update status flags.
                self.arm.update_flags()
                self.nav.update_flags()
                self.amcl.update_spin_flags()
                self.amcl.update_amcl_flags()

                if self.state == 0:
                    self.state_0_arm_tuck()
                elif self.state == 1:
                    self.state_1_wait_arm(PLANNING_TIMEOUT, EXECUTION_TIMEOUT)
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

            except Exception as e:
                import traceback
                self.node.get_logger().error(
                    f"Exception in state machine: {e}\n{traceback.format_exc()}"
                )

            time.sleep(0.05)

    
