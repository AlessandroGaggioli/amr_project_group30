import rclpy
from rclpy.node import Node
from threading import Thread, Event
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2, MoveIt2State
import time

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
]

# Joint configuration sicura per la navigazione (braccio raccolto)
# torso_lift, arm_1..7, arm_tool
HOME_JOINT_POSITIONS = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]

##############################


class Task1Manager(Node):

    def __init__(self):
        super().__init__("task1_manager")

        self.state = 0
        self.robot_base_frame = "base_link"

        self.arm_motion_started = False
        self.arm_motion_done = False
        self.arm_motion_failed = False
        self.finished = False 

        callback_group_arm = ReentrantCallbackGroup()

        self.arm = MoveIt2(
            node=self,
            joint_names=JOINT_ARM_NAMES,
            base_link_name=self.robot_base_frame,
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=callback_group_arm,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"

    def move_to_home(self):
        self.get_logger().info("Sending arm to home joint configuration")
        self.arm_motion_started = False
        self.arm_motion_done = False
        self.arm_motion_failed = False
        self.arm.move_to_configuration(
            joint_positions=HOME_JOINT_POSITIONS,
            joint_names=JOINT_ARM_NAMES,
        )

    def update_arm_flags(self):
        state = self.arm.query_state()
        self.get_logger().info(f"Arm state: {state}", throttle_duration_sec=2.0)

        if not self.arm_motion_started and state == MoveIt2State.EXECUTING:
            self.get_logger().info("Arm motion started")
            self.arm_motion_started = True

        if self.arm_motion_started and state == MoveIt2State.IDLE:
            self.get_logger().info("Arm motion finished successfully")
            self.arm_motion_done = True

        # Gestisci fallimento: planning fallito, torna IDLE senza mai eseguire
        if not self.arm_motion_started and state == MoveIt2State.IDLE and self.arm_motion_failed is False:
            pass  # ancora in attesa

    def run_state_machine(self, executor_ready: Event):
        self.get_logger().info("State machine waiting for executor...")
        executor_ready.wait()
        self.get_logger().info("Executor ready, starting state machine")
        time.sleep(10.0)

        PLANNING_TIMEOUT = 5.0  # secondi prima di considerare il planning fallito

        send_time = None

        while rclpy.ok():
            try:
                self.update_arm_flags()

                if self.state == 0:
                    self.get_logger().info("State 0: Moving arm to home joint configuration")
                    self.move_to_home()
                    send_time = time.time()
                    self.state = 1

                elif self.state == 1:
                    self.get_logger().info(
                        "State 1: Waiting for arm to reach home", throttle_duration_sec=2.0
                    )

                    if self.arm_motion_done:
                        self.get_logger().info("Arm reached home position!")
                        self.state = 2

                    # Se passa il timeout senza che inizi l'esecuzione, riprova
                    elif not self.arm_motion_started and (time.time() - send_time) > PLANNING_TIMEOUT:
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
        #rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = Task1Manager()

    executor_ready = Event()

    state_thread = Thread(
        target=node.run_state_machine,
        args=(executor_ready,),
        daemon=True
    )
    state_thread.start()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
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