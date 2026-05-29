import time
from threading import Event, Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tiago_project_group30.tiago_arm import ArmController


class Task1Manager(Node):

    def __init__(self):
        super().__init__("task1_manager")

        self.state = 0
        self.finished = False

        callback_group_arm = ReentrantCallbackGroup()
        self.arm = ArmController(self, callback_group_arm)

    def run_state_machine(self, executor_ready: Event):
        self.get_logger().info("State machine waiting for executor...")
        executor_ready.wait()
        self.get_logger().info("Executor ready, starting state machine")
        time.sleep(10.0)

        PLANNING_TIMEOUT = 5.0
        send_time = None

        while rclpy.ok():
            try:
                self.arm.update_flags()

                if self.state == 0:
                    self.get_logger().info("State 0: Moving arm to home joint configuration")
                    self.arm.move_to_home()
                    send_time = time.time()
                    self.state = 1

                elif self.state == 1:
                    self.get_logger().info(
                        "State 1: Waiting for arm to reach home",
                        throttle_duration_sec=2.0,
                    )
                    if self.arm.motion_done:
                        self.get_logger().info("Arm reached home position!")
                        self.state = 2
                    elif (
                        not self.arm.motion_started
                        and (time.time() - send_time) > PLANNING_TIMEOUT
                    ):
                        self.get_logger().warn("Planning timed out or failed, retrying...")
                        self.state = 0

                elif self.state == 2:
                    self.get_logger().info("State 2: Task completed, explore_lite can start")
                    self.finished = True
                    break

            except Exception as e:
                self.get_logger().error(f"Exception in state machine: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())

            time.sleep(0.02)

        self.get_logger().info("Task 1 Manager completed...")


def main(args=None):
    rclpy.init(args=args)
    node = Task1Manager()

    executor_ready = Event()
    state_thread = Thread(
        target=node.run_state_machine, args=(executor_ready,), daemon=True
    )
    state_thread.start()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_ready.set()

    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
