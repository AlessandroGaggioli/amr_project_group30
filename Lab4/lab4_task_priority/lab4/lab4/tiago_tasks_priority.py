import rclpy
from rclpy.node import MutuallyExclusiveCallbackGroup
from rclpy.node import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from task_priority.robot import Robot

from lab4_msg.srv import EndEffectorGoal


import numpy as np
import copy


class TaskPriorityNode(Node):
    def __init__(self):
        super().__init__("task_priority_node")

        # Grasping object and its position wrt the world(base) frame
        self.end_link = "arm_7_link"

        # Setting up TASK PRIORITY framework
        self.robot = Robot(device="cpu")
        self.chain_arm = "arm"
        self.robot.add_arm(
            "/home/student/tiago_ws/src/lab4/tiago.urdf",
            start_link="torso_lift_link",
            ee_link=self.end_link,
            task_control_name=self.chain_arm,
            skip_fixed=True,
        )

        # Adding tasks
        self.robot.add_task("EndEffectorPositionControl", 1, self.chain_arm)
        self.robot.add_task("JointLimitAvoidance", 0, chain_name=self.chain_arm)

        # Setting task params
        self.robot.set_task_param(
            "EndEffectorPositionControl",
            self.chain_arm,
            ee_target=np.array([0.6, -0.5, 0.0, 3.14, 0.0, 0.785]),
            control="POSITION",
            K=np.array([1.0, 1.0, 1.0, 1.5, 1.5, 1.5]),
            cycle_hz=100,
        )

        self.robot.set_task_param(
            "JointLimitAvoidance", self.chain_arm, limit_offsets=np.array([0.03] * 14), gains=np.array([0.3] * 14)
        )

        # Control loop timer setup
        cbg1 = MutuallyExclusiveCallbackGroup()
        self.control_timer = self.create_timer(0.01, self.control_callback, callback_group=cbg1)

        # Subscriber to joint state
        cbg2 = ReentrantCallbackGroup()
        self.joints_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, qos_profile=10, callback_group=cbg2
        )

        # Velocity publisher
        self.vel_pub = self.create_publisher(Float64MultiArray, "/arm_velocitiy_controller/commands", qos_profile=10)

        #
        self.srv = self.create_service(EndEffectorGoal, "set_task_priority_ee_goal", self.set_ee_goal)

        # Joint position array
        self.joint_positions = []

    def joint_state_callback(self, msg: JointState):
        indice = np.zeros(7)

        indice[0] = msg.name.index("arm_1_joint")
        indice[1] = msg.name.index("arm_2_joint")
        indice[2] = msg.name.index("arm_3_joint")
        indice[3] = msg.name.index("arm_4_joint")
        indice[4] = msg.name.index("arm_5_joint")
        indice[5] = msg.name.index("arm_6_joint")
        indice[6] = msg.name.index("arm_7_joint")

        self.joint_positions = [
            copy.deepcopy(msg.position[int(indice[0])]),
            copy.deepcopy(msg.position[int(indice[1])]),
            copy.deepcopy(msg.position[int(indice[2])]),
            copy.deepcopy(msg.position[int(indice[3])]),
            copy.deepcopy(msg.position[int(indice[4])]),
            copy.deepcopy(msg.position[int(indice[5])]),
            copy.deepcopy(msg.position[int(indice[6])]),
        ]

        self.joint_velocities = [
            copy.deepcopy(msg.velocity[int(indice[0])]),
            copy.deepcopy(msg.velocity[int(indice[1])]),
            copy.deepcopy(msg.velocity[int(indice[2])]),
            copy.deepcopy(msg.velocity[int(indice[3])]),
            copy.deepcopy(msg.velocity[int(indice[4])]),
            copy.deepcopy(msg.velocity[int(indice[5])]),
            copy.deepcopy(msg.velocity[int(indice[6])]),
        ]

        # print("JOINT VEL ", self.joint_velocities)

    def set_ee_goal(self, request, response):

        new_target = [
            request.position.x,
            request.position.y,
            request.position.z,
            request.roll,
            request.pitch,
            request.yaw,
        ]

        tasks_param_right = ["EndEffectorPositionControl", {"ee_target": new_target, "control": "POSITION", "cycle_hz": 100}]

        self.robot.replace_task(tasks_param_right[0], 0, chain_name=self.chain_arm)
        self.robot.set_task_param(tasks_param_right[0], chain_name=self.chain_arm, **tasks_param_right[1])

        response.result = 1
        return response

    def control_callback(self):
        if self.joint_positions != []:

            # compute new velocities
            q_d = list(
                self.robot.execute(
                    np.array(self.joint_positions), np.array(self.joint_velocities), chain_name=self.chain_arm
                )
            )

            # update velocity msg
            velocity_array = Float64MultiArray()
            velocity_array.data = [float(q) for q in q_d]

            # publish
            self.vel_pub.publish(velocity_array)
            self.robot.show_status(task_name="EndEffectorPositionControl", chain_name=self.chain_arm)


def main(args=None):
    rclpy.init(args=args)

    node = TaskPriorityNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
