# Autonomous Mobile Robotics Exam - Group 30
#
# Task 3 - sequences for pick and place.

# Grasp and Drop as list of steps.


import time

from tiago_project_group30.constants import (
    ARM_MOTION_TIMEOUT,
    ATTACH_TIMEOUT,
    GRIPPER_TIMEOUT,
)
from tiago_project_group30.common_states import Phase


class Task3Sequences:
    #------
    # Sequences  for pick and place.
    #------
    def _start_sequence(self, steps, on_complete):
        self._seq = steps
        self._seq_idx = 0
        self._step_sent = False
        self._seq_on_complete = on_complete
        self.state = Phase.RUN_SEQUENCE

    def _run_sequence(self):
        # The StateMachine calls this constantly. 
        # It runs the current step. When the step finishes (True)
        # it moves to the next one. 
        if self._seq_idx >= len(self._seq):
            return
        if self._seq[self._seq_idx]():
            self._seq_idx += 1
            self._step_sent = False
            if self._seq_idx >= len(self._seq):
                self._seq_on_complete()

    # Helper to transition the robot in back_out after pick or place.
    def _enter_back_out(self, mode):
        self._push_mode = mode
        self._send_time = time.time()
        self.state = Phase.BACK_OUT

    # Sequence of steps to grasp a cube.
    def _start_grasp_sequence(self):

        a = ARM_MOTION_TIMEOUT

        self._start_sequence([
            # 1. open the hand. 
            lambda: self._gripper_step(
                opening=True, log="GRASP: opening gripper before approach"),

            # 2. Move the arm to hover safely above the cube (pre-grasp pose).
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._pre_grasp_pose_pos,
                    quat_xyzw=self._pre_grasp_pose_quat),
                a, log="GRASP: arm to PRE-GRASP (above cube)",
                timeout_msg="Arm pre-grasp timed out, retrying"),
            
            # 3. Lower the hand straight down to the grasp pose (cartesian descent, no wrist spin).
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._grasp_pose_pos,
                    quat_xyzw=self._grasp_pose_quat, cartesian=True),
                a, log="GRASP: arm to GRASP (cartesian descent)",
                timeout_msg="Arm grasp timed out, retrying"),

            # 4. Close the hand around the cube. 
            lambda: self._gripper_step(
                opening=False, log="GRASP: closing gripper around cube",
                wait=True, gripper_timeout=GRIPPER_TIMEOUT, done_settle=0.5,
                timeout_msg="Gripper close timed out; attaching anyway"),

            # 5. Link Attacher to attach the cube to the gripper. 
            lambda: self._link_step(
                self.link_attacher.attach, ATTACH_TIMEOUT,
                fail_msg="Attach link failed; retrying attach",
                timeout_msg="Attach service timed out, retrying",
                advance_on_timeout=False),

            # 6. Lift the cube up (cartesian lift, no wrist spin).
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._pre_grasp_pose_pos,
                    quat_xyzw=self._pre_grasp_pose_quat, cartesian=True),
                a, log="GRASP: lifting arm (cartesian lift)",
                timeout_msg="Arm lift timed out, retrying",
                diag_fn=lambda: self._log_gripper_z("lift")),

            # 7. Move the arm back to home position. 
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_home(tilt_head=False),
                a, log="GRASP: tucking arm to HOME for navigation",
                timeout_msg="Arm tuck timed out, retrying",
                diag_fn=lambda: self._log_gripper_z("tuck")),
        ], on_complete=self._grasp_complete)

    # Called when the grasp sequence finishes successfully.
    def _grasp_complete(self):
        self.node.get_logger().info(
            "Carry config ready -> backing out of pick zone"
        )
        # Raise the robot's head to look ahead. 
        self.arm.tilt_head(0.0, 0.0)
        # Shift to next phase. 
        self._enter_back_out("pick")

    # Sequence of steps to drop a cube.
    def _start_drop_sequence(self):
        a = ARM_MOTION_TIMEOUT

        self._start_sequence([

            # 1. Move the arm to hover above the place surface (pre-drop pose).
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._pre_drop_pose_pos,
                    quat_xyzw=self._pre_drop_pose_quat),
                a, log="DROP: arm to PRE-DROP (above place surface)",
                timeout_msg="Arm pre-drop timed out, retrying",
                prepare=self._precompute_drop_poses,
                prepare_err="Failed to compute drop poses"),

            # 2. Lower the cube straight down to the drop pose (cartesian descent, no wrist spin). 
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._drop_pose_pos,
                    quat_xyzw=self._drop_pose_quat, cartesian=True),
                a, log="DROP: arm to DROP (cartesian descent)",
                timeout_msg="Arm drop timed out, retrying"),
            
            # 3. Open the gripper to release the cube.
            lambda: self._gripper_step(
                opening=True, log="DROP: opening gripper to release cube"),

            # 4. Detach the cube from the gripper in the link attacher.
            lambda: self._link_step(
                self.link_attacher.detach, ATTACH_TIMEOUT,
                fail_msg="Detach link failed; continuing anyway",
                timeout_msg="Detach service timed out",
                advance_on_timeout=True),

            # 5. Lift the empty hand straight up (cartesian lift, no wrist spin) to clear the dropped cube.
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_pose(
                    position=self._pre_drop_pose_pos,
                    quat_xyzw=self._pre_drop_pose_quat),
                a, log="DROP: lifting arm clear of dropped cube",
                timeout_msg="Arm lift-after-drop timed out, retrying"),

            # 6. Move the arm back to home position.
            lambda: self._arm_motion_step(
                lambda: self.arm.move_to_home(tilt_head=False),
                a, log="DROP: tucking arm to HOME for nav back",
                timeout_msg="Arm tuck-back timed out, retrying"),
        ], on_complete=lambda: self._enter_back_out("place"))
