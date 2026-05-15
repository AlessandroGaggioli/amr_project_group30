import sys
import rclpy
from rclpy.node import Node
from lab4_msg.srv import EndEffectorGoal


class TaskPriorityClient(Node):

    def __init__(self):
        super().__init__("task_priority_client")

        # pass parameters from command line as list of 6 elements
        if len(sys.argv) != 7:
            print("Usage: python3 tiago_tasks_priority_client.py x y z roll pitch yaw")
            sys.exit(1)

        self.x = float(sys.argv[1])
        self.y = float(sys.argv[2])
        self.z = float(sys.argv[3])
        self.roll = float(sys.argv[4])
        self.pitch = float(sys.argv[5])
        self.yaw = float(sys.argv[6])

        self.cli = self.create_client(EndEffectorGoal, "set_task_priority_ee_goal")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = EndEffectorGoal.Request()

    def send_request(self):
        self.req.position.x = self.x
        self.req.position.y = self.y
        self.req.position.z = self.z
        self.req.roll = self.roll
        self.req.pitch = self.pitch
        self.req.yaw = self.yaw
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    client = TaskPriorityClient()
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    print("response: ", response)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
