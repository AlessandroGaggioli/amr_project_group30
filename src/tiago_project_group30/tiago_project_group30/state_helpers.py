# Autonomous Mobile Robotics Exam - Group 30
#
# The main state machine that inherits this class must provide access to: 
    # self.node 
    # self.nav 
    # self.arm
    # self.state 
    # self._send_time
    # self.gripper 
    # self.link_attacher
    # self.tf_buffer
    # self.current_cube_id
    # self._head_scan_time
    # self._head_scan_idx

# Class StateRunners: 
    # Nav-goal helpers: _consume_nav_result, _nav_timeout_guard, _nav_to_approach
    # Arm goal tracking: _arm_goal_sent_for_state, _mark_arm_goal_sent, _clear_arm_goal_sent
    # Arm, Gripper and Link Attacher step-runners: _arm_motion_step, _gripper_step, _link_step
    # Head scan + robot pose: _advance_head_scan, _robot_xy_yaw

import time

import rclpy
import rclpy.time
from rclpy.duration import Duration

from tiago_project_group30.task2_kdl_helpers import quat_to_yaw
from tiago_project_group30.constants import (
    HEAD_SCAN_DWELL,
    HEAD_SCAN_PANS,
    HEAD_TILT_DOWN_FOR_CUBE,
)


class StateRunners:
    # ------------------------------------------------------------------
    # Nav-goal helpers
    # Send, Wait, Retry pattern shared by state 4 and state 5.
    # ------------------------------------------------------------------

    def _consume_nav_result(self):
        # Read the result of a finished nav goal exactly once.
        # Returns True if successful. False if failed. None if still active.

        if not self.nav.goal_done:
            return None
        self.nav.goal_done = False
        return self.nav.goal_succeeded
    

    def _nav_timeout_guard(self, timeout, timeout_msg):
        # Checks if the active nav goal has exceeded the allowed time. 
        # If yes, cancels the goal. 
        # Returns True if the robot is currently navigating. False otherwise.

        if self.nav.goal_active:
            if (
                not self.nav.timeout_fired
                and (time.time() - self._send_time) > timeout
            ):
                self.node.get_logger().warn(timeout_msg)
                self.nav.cancel()
                self.nav.timeout_fired = True
            return True
        return False

    def _nav_to_approach(self, pose, next_state, timeout, label,success_log, send_log, on_success=None):
        # Workflow to navigate to a target pose: 
            # Check if arrived. If yes, log and advance. 
            # Check if failed or timed out. If yes, log and retry.
            # If no goal active, send initial or retry goal. 

        res = self._consume_nav_result()

        if res is True:
            self.node.get_logger().info(success_log)
            if on_success is not None:
                on_success()
            else:
                self.state = next_state
            return True
        
        if res is False:
            self.node.get_logger().warn(f"{label} nav did not succeed, retrying")

        if self._nav_timeout_guard(
            timeout, f"{label} nav timed out, cancelling for retry"
        ):
            return True
        
        # Convert 3D pose to 2D map coordinates + yaw. 
        x, y, yaw = self.nav.approach_pose_to_xy_yaw(pose)
        self.node.get_logger().info(send_log)

        #Send nav goal and record send time for timeout tracking.
        self.nav.send_goal(x, y, yaw)
        self._send_time = time.time()

        return False

    # ---------------------
    # Arm goal tracking
    # ---------------------
    def _arm_goal_sent_for_state(self, state_id: int) -> bool:
        return getattr(self, "_arm_goal_for_state", None) == state_id

    def _mark_arm_goal_sent(self, state_id: int):
        self._arm_goal_for_state = state_id

    def _clear_arm_goal_sent(self):
        self._arm_goal_for_state = None

    # ------------------------------------------------------------------
    # Arm, Gripper and Link Attacher Helpers 
    # ------------------------------------------------------------------

    # These are STEP runners: each returns True when its step is COMPLETE
    # (the sequence driver in Task 3 advances to the next step) and False
    # while still working. They use the single self._step_sent latch, which
    # the driver resets between steps -- only one step is ever in flight.

    def _arm_motion_step(self, send_fn, arm_timeout, log=None, timeout_msg=None,
                         diag_fn=None, prepare=None, prepare_err="prepare failed"):
        # Send an arm goal, then poll motion_done with a timeout-retry.
        if not self._step_sent:
            # Optional pre-send hook (e.g. compute a pose); on failure log,
            # sleep and retry WITHOUT marking the step sent.
            if prepare is not None:
                try:
                    prepare()
                except Exception as e:
                    self.node.get_logger().error(f"{prepare_err}: {e}")
                    time.sleep(0.5)
                    return False
            if log:
                self.node.get_logger().info(log)
            send_fn()
            self._step_sent = True
            self._send_time = time.time()
            return False
        # Optional per-tick diagnostic while the motion runs.
        if diag_fn is not None:
            diag_fn()
        if self.arm.motion_done:
            return True
        if (time.time() - self._send_time) > arm_timeout:
            self.node.get_logger().warn(
                timeout_msg or "arm motion timed out, retrying"
            )
            self._step_sent = False  # re-send next tick
        return False

    def _gripper_step(self, opening, log, settle=1.0, wait=False,
                      gripper_timeout=None, done_settle=0.0, timeout_msg=None):
        # Open/close the gripper.
        #   wait=False -> fire-and-forget + a VISIBLE pause (Gazebo must render
        #                 the motion per the exam spec); done in one tick.
        #   wait=True  -> poll action_done (grasp close, so the fingers settle
        #                 before the link attach); advance on done (after
        #                 done_settle) or on timeout.
        if not wait:
            self.node.get_logger().info(log)
            (self.gripper.open if opening else self.gripper.close)()
            time.sleep(settle)
            return True
        if not self._step_sent:
            self.node.get_logger().info(log)
            (self.gripper.open if opening else self.gripper.close)()
            self._step_sent = True
            self._send_time = time.time()
            return False
        if self.gripper.action_done:
            if done_settle:
                time.sleep(done_settle)
            return True
        if (time.time() - self._send_time) > gripper_timeout:
            self.node.get_logger().warn(timeout_msg or "gripper timed out")
            return True
        return False

    def _link_step(self, action_fn, timeout, fail_msg, timeout_msg,
                   advance_on_timeout):
        # Issue an attach/detach service call, poll action_done. Advances
        # (warning on a failed result). On timeout, advance only if
        # advance_on_timeout, otherwise re-send.
        if not self._step_sent:
            action_fn(self.current_cube_id)
            self._step_sent = True
            self._send_time = time.time()
            return False
        if self.link_attacher.action_done:
            if not self.link_attacher.action_succeeded:
                self.node.get_logger().warn(fail_msg)
            return True
        if (time.time() - self._send_time) > timeout:
            self.node.get_logger().warn(timeout_msg)
            if advance_on_timeout:
                return True
            self._step_sent = False  # re-send next tick
        return False

    # ------------------------------------------------------------------
    # Head scan + robot pose
    # ------------------------------------------------------------------
    def _advance_head_scan(self, log_prefix):

        # Moves the robot's head through a sequence of pan angles to scan the environment.
        # It waits each angle for HEAD_SCAN_DWELL seconds. 

        if (time.time() - self._head_scan_time) >= HEAD_SCAN_DWELL:

            self._head_scan_idx = (self._head_scan_idx + 1) % len(HEAD_SCAN_PANS)
            pan = HEAD_SCAN_PANS[self._head_scan_idx]

            self.node.get_logger().info(
                f"{log_prefix} pan={pan:.2f} rad",
                throttle_duration_sec=1.0,
            )

            self.arm.tilt_head(HEAD_TILT_DOWN_FOR_CUBE, pan)

            self._head_scan_time = time.time()

    def _robot_xy_yaw(self, timeout=0.5):
        # Ask the TF tree for the robot's current position and rotation on the map. 
        # Returns (X, Y, Yaw). 

        tf = self.tf_buffer.lookup_transform(
            "map", "base_link",
            rclpy.time.Time(),
            timeout=Duration(seconds=timeout),
        )

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        yaw = quat_to_yaw(
            0.0, 0.0, tf.transform.rotation.z, tf.transform.rotation.w
        )

        return rx, ry, yaw
