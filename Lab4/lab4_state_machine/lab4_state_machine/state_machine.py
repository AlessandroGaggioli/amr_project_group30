from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, GripperInterface


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

JOINT_GRIPPER_NAMES = [
    "gripper_left_finger_joint",
    "gripper_right_finger_joint",
]

OPEN_GRIPPER_JOINT_POSITIONS = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS = [0.01, 0.01]


##############################


class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine_node')

        self.state = 0 # Initial state

        self.robot_base_frame = "base_link"

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group_arm = ReentrantCallbackGroup()
        callback_group_gripper = ReentrantCallbackGroup()


        # arm
        self.arm = MoveIt2(
            node=self,
            joint_names=JOINT_ARM_NAMES,
            base_link_name=self.robot_base_frame,
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=callback_group_arm,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"

        
        # gripper
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=JOINT_GRIPPER_NAMES,
            open_gripper_joint_positions=OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name="gripper",
            callback_group=callback_group_gripper,
            gripper_command_action_name="gripper_controller/joint_trajectory",
        )


        # Spin the node in background thread(s) and wait a bit for initialization
        executor = rclpy.executors.MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Sleep a while in order to get the first joint state
        self.create_rate(10.0).sleep()

        # Define target poses
        self.target_pose = {
            "position": [0.3, -0.4, 0.6],  # Example target position
            "orientation": [0.731, -0.001, -0.682, 0.026]  # Example quaternion orientation
        }
        self.up_pose = {
            "position": [0.3, -0.4, 0.75],  # Move up after grasping
            "orientation": [0.731, -0.001, -0.682, 0.026]
        }



    def move_gripper(self, pos):
        self.gripper.move_to_position(pos)
        print("waiting...")
        self.gripper.wait_until_executed()

    def move_to_pose(self, pos, quat):
        self.arm.move_to_pose(position=pos, quat_xyzw=quat)
        self.arm.wait_until_executed()


    def run_state_machine(self):
        while rclpy.ok():

            if self.state == 0:
                self.get_logger().info("State 0: Moving to up pose (approach)")
                self.move_to_pose(self.up_pose["position"], self.up_pose["orientation"])
                self.state = 1

            elif self.state == 1:
                self.get_logger().info("State 1: Open gripper")
                self.move_gripper(0.035)
                self.state = 2

            elif self.state == 2:
                self.get_logger().info("State 2: Moving to target pose")
                self.move_to_pose(self.target_pose["position"], self.target_pose["orientation"])
                self.state = 3

            elif self.state == 3:
                self.get_logger().info("State 3: Closing gripper")
                self.move_gripper(0.01)
                self.state = 4

            elif self.state == 4:
                self.get_logger().info("State 4: Moving up")
                self.move_to_pose(self.up_pose["position"], self.up_pose["orientation"])
                self.state = 5

            elif self.state == 5:
                self.get_logger().info("State 5: Task completed")
                break


def main(args=None):
    rclpy.init(args=args)

    tiago_move_node = StateMachineNode()
        
    # Begin state machine
    tiago_move_node.run_state_machine()

    rclpy.spin(tiago_move_node)
    tiago_move_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
