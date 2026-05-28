from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import GripperInterface


##############################
# Tiago Parameters
JOINT_NAMES = [
    "gripper_left_finger_joint",
    "gripper_right_finger_joint",
]

OPEN_GRIPPER_JOINT_POSITIONS = [0.04, 0.04]

CLOSED_GRIPPER_JOINT_POSITIONS = [0.0, 0.0]

GRIPPER_GROUP_NAME = "gripper"

GRIPPER_COMMAND_ACTION_NAME = "gripper_controller/joint_trajectory"

##############################


def main():
    rclpy.init()

    # Create node for this example
    node = Node("example_tiago_gripper")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create gripper interface
    gripper_interface = GripperInterface(
        node=node,
        gripper_joint_names=JOINT_NAMES,
        open_gripper_joint_positions=OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=GRIPPER_GROUP_NAME,
        callback_group=callback_group,
        gripper_command_action_name=GRIPPER_COMMAND_ACTION_NAME,
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Perform gripper action
    node.get_logger().info("Opening gripper")
    gripper_interface.open()
    gripper_interface.wait_until_executed()

    node.get_logger().info("Closing gripper")
    gripper_interface.close()
    gripper_interface.wait_until_executed()

    # Move to a specific position
    node.get_logger().info("Moving gripper to position 0.02")
    gripper_interface.move_to_position(0.02)
    gripper_interface.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
