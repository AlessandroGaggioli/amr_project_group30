from task_priority.Tasks.NTkineLib.robot_kins.chain_utils.robot_components.transforms.quaternion import Quaternion
from task_priority.Tasks.task import Task
from enum import Enum
import numpy as np


class EndEffectorControl(Task):
    class Control(Enum):
        VELOCITY = 1
        POSITION = 2

    def __init__(self, priority, name, num_of_joints, device="cpu", dtype=np.float32):
        self.device = device
        self.dtype = dtype
        super().__init__(name, priority, num_of_joints, device, dtype)
        self.integrated_error = np.array([0, 0, 0], dtype=self.dtype)
        self.u = np.array([[0, 0, 0]], dtype=self.dtype)
        self.quat_operations = Quaternion(dtype=dtype)
        self.K = np.ones(6, dtype=self.dtype)
        self.A_ = np.eye(6, dtype=self.dtype)
        self.cycleHz = None
        self.target_pos = None
        self.target_quat = None

        self._task_completed = False

        self.control_dimensions = ["x", "y", "z", "roll", "pitch", "yaw"]
        # self.csv_file = open('/ros1_docker/ros1_ws/src/data/error_pos_ee_6.csv', mode='w')

    def set_param(self, **kwargs):
        target_task = kwargs.get("ee_target")
        current_dimensions = kwargs.get("control_dimensions", self.control_dimensions)
        self.missing_dimensions = [i for i, dim in enumerate(self.control_dimensions) if dim not in current_dimensions]
        cycleHz = kwargs.get("cycle_hz")

        self.A_ = np.eye(len(current_dimensions), dtype=self.dtype)  # da testare bisogna ragionarci

        self.K = kwargs.get("K", self.K)

        self.target_pos = np.array(target_task[:3], dtype=self.dtype)
        self.target_quat = self.quat_operations.from_rpy(target_task[3], target_task[4], target_task[5])
        self.cycleHz = cycleHz

    def update_task(self, **kwargs):
        actual_pose = kwargs.get("actual_pose")
        self.jacobian = np.delete(self.jacobian, self.missing_dimensions, axis=0)
        self.controller(actual_pose)

    def get_JK(self):
        return self.jacobian

    def task_completed(self):
        return self._task_completed

    def controller(self, actual_pose):
        """
        actual_pose: 4x4 torch tensor rapresenting the homogeneous transformation matrix of the end effector
        actual_vel: 6x1 torch tensor rapresenting the velocity vector of the end effector
        """

        actual_pos = actual_pose[:3]
        self.quat_operations.from_rpy(actual_pose[3], actual_pose[4], actual_pose[5])

        self.error = self.target_pos - actual_pos
        u_position = self.K[:3] * self.error.copy()

        if abs(u_position[0]) < 1e-2 and abs(u_position[1]) < 1e-2 and abs(u_position[2]) < 1e-2:
            u_position = np.zeros(3, dtype=self.dtype)

        self.w_target = self.quat_operations.rotation_velocity(self.target_quat, self.cycleHz)

        if abs(self.w_target[0]) < 1e-2 and abs(self.w_target[1]) < 1e-2 and abs(self.w_target[2]) < 1e-2:
            self.w_target = np.zeros(3, dtype=self.dtype)

        self._task_completed = (
            np.linalg.norm(self.target_pos - actual_pos) < 1e-2 and np.linalg.norm(self.w_target) < 1e-2
        )

        u_rot = self.K[3:6] * self.w_target

        self.dx = np.concatenate((u_position, u_rot), axis=0)
        self.dx = np.delete(self.dx, self.missing_dimensions, axis=0)

    def show(self, **kwargs):
        # Nome del task in blu
        print("\n\033[94m-- EE Control Status --\033[0m")

        # Verifica se il task è completato
        pos_completed = abs(self.error[0]) < 1e-2 and abs(self.error[1]) < 1e-2 and abs(self.error[2]) < 1e-2
        orient_completed = (
            abs(self.w_target[0]) < 1e-2 and abs(self.w_target[1]) < 1e-2 and abs(self.w_target[2]) < 1e-2
        )

        if pos_completed and orient_completed:
            print("\033[92mIl task è completato: il robot è entro i limiti del target\033[0m")
        else:
            # Errore di posizione se non è completato
            if not pos_completed:
                print(f"\033[93mErrore di posizione:\033[0m")  # Giallo
                print(f"\033[93m- Asse X: {self.error[0]:.4f} metri\033[0m")
                print(f"\033[93m- Asse Y: {self.error[1]:.4f} metri\033[0m")
                print(f"\033[93m- Asse Z: {self.error[2]:.4f} metri\033[0m")

            # Errore di orientamento se non è completato
            if not orient_completed:
                print(f"\033[93mErrore di orientamento:\033[0m")  # Giallo
                print(f"\033[93m- Roll (X): {self.w_target[0]:.4f} radianti\033[0m")
                print(f"\033[93m- Pitch (Y): {self.w_target[1]:.4f} radianti\033[0m")
                print(f"\033[93m- Yaw (Z): {self.w_target[2]:.4f} radianti\033[0m")
