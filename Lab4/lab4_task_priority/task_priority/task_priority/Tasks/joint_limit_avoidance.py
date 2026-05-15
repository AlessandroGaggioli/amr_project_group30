import math
import numpy as np
from task_priority.Tasks.task import Task


class JointLimitAvoidance(Task):
    # def __init__(self, priority:int, name:str, num_of_joints:int, up_limts:list, lower_limits:list, device='cpu', dtype=torch.float32):
    def __init__(
        self, priority: int, name: str, num_of_joints: int, lower_lim, upper_lim, device="cpu", dtype=np.float32
    ):
        super().__init__(name, priority, num_of_joints, device, dtype)
        self.upper_lim = upper_lim
        self.lower_lim = lower_lim
        self.m = 2 * num_of_joints
        # merge the lower and upper limits in a single tensor
        self._q_lim = np.empty((0, 2), dtype=self.dtype)

        # for i in range(len(lower_lim)):
        #     self._q_lim = np.concatenate((self._q_lim, np.array([[upper_lim[i], lower_lim[i]]], dtype=self.dtype)))

        self._q_lim = np.array(
            [upper_lim[i] if j % 2 == 0 else lower_lim[i] for i in range(num_of_joints) for j in range(2)]
        )

        print("\033[95mUpper Joint limits: ", self.upper_lim, "\033[0m")
        print("\033[95mLower Joint limits: ", self.lower_lim, "\033[0m")

        self.limit_offsets = 0.1 * np.ones(num_of_joints * 2, dtype=self.dtype)

        self.K = 0.1 * np.ones(num_of_joints * 2, dtype=self.dtype)
        self.A_ = np.zeros((num_of_joints * 2, num_of_joints * 2), dtype=self.dtype)

        self.x = np.zeros(num_of_joints * 2, dtype=self.dtype)

        self.xd = np.zeros(num_of_joints * 2, dtype=self.dtype)

        self.num_of_joints = num_of_joints

        self.jacobian = np.zeros((num_of_joints * 2, num_of_joints), dtype=self.dtype)
        for i in range(num_of_joints):
            self.jacobian[2 * i, i] = 1
            self.jacobian[2 * i + 1, i] = 1


    def set_param(self, **kwargs):
        self.limit_offsets = kwargs.get(
            "limit_offsets", self.limit_offsets
        )  # takes the value of b from the dictionary, if not it takes the value of self.limit_offsets in constructor
        self.K = kwargs.get(
            "K", self.K
        )  # takes the value of gains from the dictionary, if not it takes the value of self.K in constructor
        self.upper_lim = kwargs.get("upper_lim", self.upper_lim)
        self.lower_lim = kwargs.get("lower_lim", self.lower_lim)

        self._q_lim = np.array(
            [
                self.upper_lim[i] if j % 2 == 0 else self.lower_lim[i]
                for i in range(self.num_of_joints)
                for j in range(2)
            ]
        )

        if isinstance(self.limit_offsets, list):
            self.limit_offsets = np.array(self.limit_offsets, dtype=self.dtype)
        if isinstance(self.K, list):
            self.K = np.array(self.K, dtype=self.dtype)
        if isinstance(self._q_lim, list):
            self._q_lim = np.array(self._q_lim, dtype=self.dtype)

        print("\033[95mUpper Joint limits: ", self.upper_lim, "\033[0m")
        print("\033[95mLower Joint limits: ", self.lower_lim, "\033[0m")
        print("\033[95mLimit offsets: ", self.limit_offsets, "\033[0m")
        print("\033[95mGains: ", self.K, "\033[0m")

    def get_JK(self):
        return self.jacobian

    def update_constraints(self, joint_pos):
        # print("\033[95mJoint positions: ", joint_pos, "\033[0m")
        for i in range(self.num_of_joints):
            self.xd[2 * i] = self._q_lim[2 * i] - self.limit_offsets[2 * i]
            self.xd[2 * i + 1] = self._q_lim[2 * i + 1] + self.limit_offsets[2 * i + 1]
            self.x[2 * i] = joint_pos[i]
            self.x[2 * i + 1] = joint_pos[i]

    def update_A(self):
        for i in range(self.m):
            if i % 2 == 0:
                self.A_[i, i] = self.sigmoid(self.x[i], self._q_lim[i] - self.limit_offsets[i], self.limit_offsets[i])
            else:
                self.A_[i, i] = 1 - self.sigmoid(
                    self.x[i], self._q_lim[i] + self.limit_offsets[i], self.limit_offsets[i]
                )

        # print("\033[95mA_: ", self.A_, "\033[0m")

    def update_jacobian(self, jac=None):
        pass

    def controller(self):
        self.dx = self.K * (self.xd - self.x)
        # print("\033[95mJoint limit avoidance: ", self.dx, "\033[0m")

    def update_task(self, **kwarg):
        self.update_constraints(kwarg.get("joint_pos"))
        self.update_A()
        self.controller()

    def sigmoid(self, x, xm, b):
        # print("x: ", x, "xm: ", xm, "b: ", b)
        if x > xm:
            return 1.0
        if x < (xm + b):
            return 0.0
        return (math.cos((x - xm) * math.pi / b) + 1) / 2

    def to(self, device, dtype):
        super().to(device, dtype)
        self._q_ref = self._q_ref.to(device=device, dtype=dtype)
        return self

    def show(self, **kwargs):
        print("\n\033[94m-- Joint Limit Avoidance Status --\033[0m")

        for i in range(self.num_of_joints):
            # Controlla se il giunto i è fuori limite superiore o inferiore
            upper_limit_violation = self.A_[2 * i, 2 * i] > 0
            lower_limit_violation = self.A_[2 * i + 1, 2 * i + 1] > 0

            if upper_limit_violation or lower_limit_violation:

                # Stampa quale giunto è fuori dai limiti e quale limite è violato
                over_upper = self.x[2 * i] - (self._q_lim[2 * i] - self.limit_offsets[2 * i])
                if upper_limit_violation:
                    print(f"\033[93mJoint {i + 1} è fuori dal limite superiore di {over_upper}\033[0m")
                if lower_limit_violation:
                    under_lower = (self._q_lim[2 * i + 1] + self.limit_offsets[2 * i + 1]) - self.x[2 * i + 1]
                    print(f"\033[93mJoint {i + 1} è fuori dal limite inferiore di {under_lower} \033[0m")
            else:
                # Se non ci sono violazioni, stampa che il giunto è dentro i limiti
                print(f"Joint {i + 1} è dentro i limiti")
