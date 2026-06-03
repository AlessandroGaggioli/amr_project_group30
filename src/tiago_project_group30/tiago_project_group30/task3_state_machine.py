# Autonomous Mobile Robotics Exam - Group 30

# Task 3 - main state machine.
#
# The class body is split across several modules for readability; this file keeps only
# __init__, current_cube_id and the main dispatch loop:
#   Task3Phases    (task3_phases.py)     -- nav overrides + phase handlers
#   Task3Sequences (task3_sequences.py)  -- RUN_SEQUENCE driver + grasp/drop
#   Task3Geometry  (task3_geometry.py)   -- pose math + cmd_vel push helpers
#
# Flow:
#   ARM_HOME -> WAIT_ARM -> AMCL -> SEARCH -> NAV_PICK
#   NAV_PICK reached -> HEAD_TILT -> WAIT_CUBE
#       (first detection) -> WAIT_CUBE_NAV -> PUSH(pick) -> WAIT_CUBE
#       (close detection) -> RUN_SEQUENCE(grasp) -> BACK_OUT(pick) -> NAV_PLACE
#   NAV_PLACE reached -> REFRESH_PLACE -> PUSH(place)
#       -> RUN_SEQUENCE(drop) -> BACK_OUT(place) -> NEXT_OR_DONE
#   NEXT_OR_DONE -> NAV_PICK (next cube) or DONE.

import time
from threading import Event

import rclpy

from tiago_project_group30.constants import (
    ARM_MOTION_TIMEOUT,
    APPROACH_NAV_TIMEOUT,
    CUBE_DETECTION_TIMEOUT,
    CUBE_PICK_SEQUENCE,
    PLANNING_TIMEOUT,
    SEARCH_NAV_TIMEOUT,
)
from tiago_project_group30.common_states import CommonStates, Phase
from tiago_project_group30.task3_phases import Task3Phases
from tiago_project_group30.task3_sequences import Task3Sequences
from tiago_project_group30.task3_geometry import Task3Geometry


class Task3StateMachine(Task3Phases, Task3Sequences,
                        Task3Geometry, CommonStates):

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
        self._current_cube_idx = 0   # which cube the robot is working on. 

        # Memory variables for cube poses and grasp/place poses. 
        self._cube_approached = False 
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

        # To handle step by step pick and place. 
        self._seq = []
        self._seq_idx = 0
        self._seq_on_complete = None
        self._step_sent = False

    @property # retunrn the ID of the cube currently being processed. 
    def current_cube_id(self) -> int:
        return CUBE_PICK_SEQUENCE[self._current_cube_idx]

    # -------------
    # Main loop
    # -------------
    def run(self, executor_ready: Event):

        self.node.get_logger().info("State machine: waiting for executor...")
        executor_ready.wait()
        self.node.get_logger().info("Executor ready, giving the stack 10s to come up")
        time.sleep(10.0)

        while rclpy.ok() and not self.finished:
            try:
                self.arm.update_flags()
                self.nav.update_flags()
                self.amcl.update_spin_flags()
                self.amcl.update_amcl_flags()
                self.gripper.update_flags()

                phase = self.state

                if phase == Phase.ARM_HOME:
                    self.state_0_arm_tuck()
                elif phase == Phase.WAIT_ARM:
                    self.state_1_wait_arm(PLANNING_TIMEOUT, ARM_MOTION_TIMEOUT)
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
