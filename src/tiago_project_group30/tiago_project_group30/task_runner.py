# Autonomous Mobile Robotics Exam - Group 30
#
# Shared functions for tasks. 
# The state machine runs in the background on a separate thread.
# Meanwhile the MultiThreadedExecutor keeps the ROS2 node active
# Process incoming messages, services and actions so they never block the loop.

from threading import Event, Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor


def run_task(node, state_machine, num_threads=4):
    executor_ready = Event()
    Thread(
        target=state_machine.run, args=(executor_ready,), daemon=True
    ).start()

    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(node)
    executor_ready.set()

    try:
        while rclpy.ok() and not state_machine.finished:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
