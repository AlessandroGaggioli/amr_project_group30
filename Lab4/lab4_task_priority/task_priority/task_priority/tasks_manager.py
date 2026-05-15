import time
import numpy as np
from task_priority.Tasks.NTkineLib.base_arm_gripper import RobotDynKins

from task_priority.Tasks.ee_pos_ctrl import EndEffectorControl
from task_priority.Tasks.joint_limit_avoidance import JointLimitAvoidance


"""
parameter for the task of control ee
    ee_position: list of 6 elements with the position of the end effector
    control: string with the type of control, POSITION or VELOCITY
    cycle_hz: int with the frequency of the control
"""


class TaskManager:
    def __init__(self, device="cpu", dtype=np.float32, skip_fixed=True) -> None:
        # take the folder of urdf

        self.device = device
        self.dtype = dtype
        self.robots = RobotDynKins(device=device, dtype=dtype)
        self.num_joints = 0
        self.joint_names = []
        self.link_names = []
        self.skip_fixed = skip_fixed

        print("\033[92m" + "Number of joints: {}".format(self.num_joints) + "\033[0m")

        self.tasks = []
        # self.Q           = np.eye(6,6,dtype=self.dtype)
        self.eta = 0.001
        self.v_lin = np.zeros(6, dtype=self.dtype)
        self._foward = np.zeros((4, 4), dtype=self.dtype)

        self.dqd = np.zeros(self.num_joints, dtype=self.dtype)
        self.Q = np.eye(self.num_joints, self.num_joints, dtype=self.dtype)
        self.robot_override = 100

    def add_arm(self, urdf_path, start_link, ee_link, tcp_link=None):
        self.robots.add_arm(urdf_path, start_link, ee_link, tcp_link)
        self.__update_robot_params()

    def add_gripper(self, urdf_path, start_link, tip_link_names):
        self.robots.add_gripper(urdf_path, start_link, tip_link_names)
        self.__update_robot_params()

    def add_base(self, wheels, wheels_type="omni"):
        pass

    def __update_robot_params(self):
        self.num_joints, self.num_links = self.robots.num_of_joints, self.robots.num_of_links
        self.joint_names = self.robots.joint_names
        self.link_names = self.robots.link_names
        self.joint_limits_low, self.joint_limits_upper = self.robots.joint_limits_low, self.robots.joint_limits_upper
        self.joint_vel_limits = np.array(self.robots.joint_vel_lim)

        # print("\033[92m" + 'Number of joints: {}'.format(self.num_joints) + "\033[0m")
        # print("\033[92m" + 'Joint names: {}'.format(self.joint_names) + "\033[0m")
        # print("\033[92m" + 'Link names: {}'.format(self.link_names) + "\033[0m")
        # print("\033[92m" + 'Joint limits: {}'.format(self.joint_limits_low) + "\033[0m")
        # print("\033[92m" + 'Joint limits: {}'.format(self.joint_limits_upper) + "\033[0m")
        # print("\033[92m" + 'Joint velocity limits: {}'.format(self.joint_vel_limits) + "\033[0m")
        # print("\033[92m" + 'Number of links: {}'.format(self.num_links) + "\033[0m")

    def add_task(self, task_name, priority):

        if task_name == "EndEffectorPositionControl":
            task = EndEffectorControl(priority, task_name, self.num_joints, device=self.device, dtype=self.dtype)
            self.tasks.append(task)
            # print in green that the task has been added, with the name of the task
            print("\033[92mTask added successfully: ", task_name, "with priority: ", priority, "\033[0m")
        elif task_name == "JointLimitAvoidance":
            lower_lim, upper_lim = self.joint_limits_low, self.joint_limits_upper
            task = JointLimitAvoidance(
                priority, task_name, self.num_joints, lower_lim, upper_lim, device=self.device, dtype=self.dtype
            )
            self.tasks.append(task)
            print("\033[92mTask added successfully: ", task_name, "with priority: ", priority, "\033[0m")
        elif task_name == "OtherTaskName":
            pass

        self.tasks.sort(key=lambda x: x.priority)

    def get_joint_vel_limit(self):
        return np.array(self.robots.get_joint_vel_lim())

    def remove_task(self, task_name):
        for task in self.tasks:
            if task.name == task_name:
                self.tasks.remove(task)
                return
        raise ValueError("The task name is not in the list of tasks")

    def replace_task(self, task_name, priority):
        for task in self.tasks:
            if task.name == task_name:
                self.tasks.remove(task)
                self.add_task(task_name, priority)
                return
        raise ValueError("The task name is not in the list of tasks")

    def set_override(self, override):
        self.robot_override = override

    def get_joint_limits(self, task_or_urdf="urdf"):
        if task_or_urdf == "urdf":
            for task in self.tasks:
                if isinstance(task, JointLimitAvoidance):
                    return task.lower_lim, task.upper_lim
        elif task_or_urdf == "chain":
            return self.robots.get_manipulator_joint_limits_low(), self.robots.get_manipulator_joint_limits_upper()

    def get_joint_names(self):
        return self.joint_names

    def show_status(self, task_name=None, **kwargs):
        for task in self.tasks:
            if task_name is None:
                task.show(**kwargs)
            if task.name == task_name:
                task.show(**kwargs)
                return

    def set_task_param(self, task_name, **kwargs):
        for task in self.tasks:
            if task.name == task_name:
                task.set_param(**kwargs)
                return
        raise ValueError("The task name is not in the list of tasks")

    def is_in_position(self):
        for task in self.tasks:
            if task.name == "EndEffectorControl":
                return task.task_completed()

    def get_foward_kinematics(self, joint_pos, as_dict=True):
        return self.robots.get_manipulator_fk(joint_pos, as_dict)

    def execute(
        self, joint_pos=None, joint_vel=np.zeros(6), force=np.zeros(6), odom=np.zeros(7), coll_point_cloud=None
    ):
        self.robots.compute(joint_pos)
        forward = self.robots.get_ee_vector()
        self.forward_link = self.robots.get_forward_as_dict(output_type="torch")
        # self.base.compute(odom, joint_pos_base, joint_vel_base)

        # forward = self.base.odom_matrix @ self.robots.forward_tensor[-1]
        # print(self.robots.jacobian_matrix[:3, :])

        jacobian = self.robots.jacobian_stack()
        # print("Jac Full:", jacobian)

        # joint_vel = np.concatenate((self.base.get_joint_vel_base_manipulator(), joint_vel_manipulator))
        ee_vel = jacobian @ joint_vel
        # ee_vel= np.zeros(6)

        self.dqd = np.zeros(self.num_joints, dtype=self.dtype)
        self.Q = np.eye(self.num_joints, self.num_joints, dtype=self.dtype)
        self.I = np.eye(self.num_joints, self.num_joints, dtype=self.dtype)

        for task in self.tasks:
            task.update_jacobian(jacobian)
            task.update_task(
                actual_pose=forward,
                actual_vel=ee_vel,
                force=force,
                joint_pos=joint_pos,
                joint_vel=joint_vel,
                forward_dict=self.forward_link,
                additional_points=coll_point_cloud,
            )
            JkQk_1 = task.get_JK() @ self.Q
            wnpJkQk_1Q = self.weighted_normalized_pinv(JkQk_1, task.A_, self.Q)
            wnpJkQk_1I = self.weighted_normalized_pinv(JkQk_1, task.A_, self.I)
            Wk = JkQk_1 @ wnpJkQk_1Q
            Qk = self.Q @ self.I - (wnpJkQk_1I @ JkQk_1)
            Tk = self.I - (self.Q @ (wnpJkQk_1I) @ Wk @ task.get_JK())

            self.dqd = Tk @ self.dqd + self.Q @ wnpJkQk_1I @ Wk @ task.dx
            self.Q = Qk

        return self.dqd

    def weighted_normalized_pinv(self, X, A, Q):
        # Initialize P and I as identity matrices

        P = np.eye(X.shape[1], X.shape[1], dtype=self.dtype)
        I = np.eye(Q.shape[0], Q.shape[1], dtype=self.dtype)

        # Compute tmpX
        # print("X: ", X.shape)
        # print('A', A.shape)
        # print('Q', Q.shape)
        tmpX = X.T @ A @ X + self.eta * (I - Q).T @ (I - Q)

        # Perform SVD

        U, _, V = np.linalg.svd(tmpX, full_matrices=False)
        V = V.T
        S = U.T @ tmpX @ V

        # Recompute S with the gbellmf function
        for i in range(S.shape[0]):
            P[i, i] = self.gbellmf(S[i, i] * pow(10, 6), 0.2, 1.0, 0.0)

        # Compute AA
        AA = tmpX + V @ P @ V.T

        return np.linalg.pinv(AA) @ X.T @ A @ A

    def gbellmf(self, x, a, b, c):
        return 1 / (1 + pow(abs((x - c) / a), 2 * b))
