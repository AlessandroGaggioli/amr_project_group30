from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2

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
    node = Node("example_add_collision_obj")

    obj_type = "BOX"
    obj_id = "10"
    position = [0.5, 0.0, 0.5]
    quat_xyzw = [1.0, 0.0, 0.0, 0.0]
    dimensions = [0.1, 0.1, 0.1]

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

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    if obj_type == "BOX":
        moveit2.add_collision_box(id=obj_id, position=position, quat_xyzw=quat_xyzw, size=dimensions)
    elif obj_type == "SPHERE":
        moveit2.add_collision_sphere(id=obj_id, position=position, quat_xyzw=quat_xyzw, radius=dimensions[0])
    elif obj_type == "CYLINDER":
        moveit2.add_collision_cylinder(
            id=obj_id, position=position, quat_xyzw=quat_xyzw, radius=dimensions[0], height=dimensions[1]
        )
    else:
        node.get_logger().error("Invalid object type")
        rclpy.shutdown()
        executor_thread.join()
        exit(1)

    node.get_logger().info("Collision object added")

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
