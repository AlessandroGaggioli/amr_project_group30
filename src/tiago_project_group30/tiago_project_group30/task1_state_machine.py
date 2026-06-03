# Autonomous Mobile Robotics Exam - Group 30
#
# Task1 - map generation 

# Handles the initial setup for task1. 
# Move the robot's arm to the home position using shared states (CommonStates). 
# Once the arm is tucked, the state machine finishes. 

import time
from threading import Event

import rclpy

from tiago_project_group30.common_states import CommonStates, Phase
from tiago_project_group30.constants import (
    PLANNING_TIMEOUT,
    EXECUTION_TIMEOUT,
)


class Task1StateMachine(CommonStates):
    def __init__(self, node, arm):
        self.node = node
        self.arm = arm
        self.state = 0
        self.finished = False
        self._send_time = None

    def run(self, executor_ready: Event):
        self.node.get_logger().info("State machine: waiting for executor...")

        executor_ready.wait() # wait for the ROS2 executor signal.

        self.node.get_logger().info(
            "Executor ready, giving the stack 10s to come up"
        )
        # Allow the ROS2 stack 10 seconds to fully start. 
        time.sleep(10.0)

        # Main state machine loop
        while rclpy.ok() and not self.finished:
            try:
                self.arm.update_flags() # update the current status flags. 

                phase = self.state

                if phase == Phase.ARM_HOME :#send the command to homing the arm. 
                    self.state_0_arm_tuck() 

                elif phase == Phase.WAIT_ARM: # wait for the completion of the movement.
                    self.state_1_wait_arm(PLANNING_TIMEOUT, EXECUTION_TIMEOUT) 

                elif phase == Phase.AMCL: # Arm is tucked. 
                    self.node.get_logger().info(
                        "State 2: arm at HOME, Task 1 done -- "
                        "explore_lite start"
                    )
                    self.finished = True
                    break

            except Exception as e:
                import traceback
                self.node.get_logger().error(
                    f"Exception in state machine: {e}\n{traceback.format_exc()}"
                )

            time.sleep(0.05) 
