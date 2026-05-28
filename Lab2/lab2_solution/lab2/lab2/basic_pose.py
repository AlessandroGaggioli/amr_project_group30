from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2

import PyKDL as kdl

##############################
# Tiago Parameters

JOINT_NAMES = [
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

BASE_LINK_NAME = "base_link"

END_EFFECTOR_NAME = "arm_tool_link"

GROUP_NAME = "arm_torso"

##############################


def main():
    rclpy.init()

    # Create node for this example
    node = Node("example_tiago_pose")

    position = [0.5, 0.0, 1.0]
    quat_xyzw = [1.0, 0.0, 0.0, 0.0]
    
    rot = kdl.Rotation.Quaternion(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3])
    pose = kdl.Frame(rot, kdl.Vector(position[0], position[1], position[2]))

    print(pose)

    quat_xyzw = pose.M.GetQuaternion()

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK_NAME,
        end_effector_name=END_EFFECTOR_NAME,
        group_name=GROUP_NAME,
        callback_group=callback_group,
    )
    moveit2.planner_id = "RRTConnectkConfigDefault"

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 1.0

    # Move to pose
    node.get_logger().info(f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}")
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=True)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
