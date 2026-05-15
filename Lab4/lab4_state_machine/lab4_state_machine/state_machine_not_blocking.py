from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State


##############################
# Tiago Parameters

JOINT_ARM_NAMES = [
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
    "arm_tool_joint",
]

##############################


class StateMachineNode(Node):

    def __init__(self):
        super().__init__("state_machine_node")

        self.state = 0  # Initial state
        self.robot_base_frame = "base_link"

        # Flags for arm status
        self.arm_motion_started = False
        self.arm_motion_done = False

        # Create callback groups that allow parallel execution
        callback_group_arm = ReentrantCallbackGroup()
        callback_group_gripper = ReentrantCallbackGroup()

        # Arm
        self.arm = MoveIt2(
            node=self,
            joint_names=JOINT_ARM_NAMES,
            base_link_name=self.robot_base_frame,
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=callback_group_arm,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"

        self.up_pose = {
            "position": [0.3, -0.4, 0.75],
            "orientation": [0.731, -0.001, -0.682, 0.026],
        }

    def move_to_pose(self, pos, quat):
        """
        Send arm goal and return immediately.
        """
        self.get_logger().info(f"Sending arm goal to position {pos}")
        self.arm_motion_started = False
        self.arm_motion_done = False

        self.arm.move_to_pose(position=pos, quat_xyzw=quat)

    def update_arm_flags(self):
        state = self.arm.query_state()

        # 1) detect when motion starts
        if not self.arm_motion_started and state == MoveIt2State.EXECUTING:
            self.get_logger().info("Arm motion started")
            self.arm_motion_started = True

        # 2) after it started, detect when it finishes
        if self.arm_motion_started and state != MoveIt2State.EXECUTING:
            self.get_logger().info(f"Arm motion finished, state: {state}")
            self.arm_motion_done = True
        
        # 3) Maybe improve here to check if we succeeded or not reaching the goal
        # ....

    # ---------- state machine ----------
    def run_state_machine(self):
        
        rate = self.create_rate(50.0)  # 50 Hz loop
        
        while rclpy.ok():
            # update arm flags in each iteration
            self.update_arm_flags()

            if self.state == 0:
                self.get_logger().info("State 0: Moving to up pose (approach)")
                self.move_to_pose(self.up_pose["position"], self.up_pose["orientation"])
                self.state = 1

            elif self.state == 1:
                self.get_logger().info("State 1: Waiting for arm to reach up pose")
                if self.arm_motion_done:
                    self.get_logger().info("Arm reached up pose, going to next state")
                    self.state = 2

            elif self.state == 2:
                self.get_logger().info("State 2: Task completed")
                break

            rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    tiago_move_node = StateMachineNode()

    # Run the state machine loop in a separate thread
    state_thread = Thread(target=tiago_move_node.run_state_machine, daemon=True)
    state_thread.start()

    # ROS 2 executor in main thread (multi-threaded for callbacks)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(tiago_move_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        tiago_move_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
