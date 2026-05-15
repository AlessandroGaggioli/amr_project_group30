from task_priority.tasks_manager import TaskManager
import numpy as np
from task_priority.Tasks.NTkineLib.robot_kins.chain_utils.robot_components.transforms.transforms import homogeneousMatrix_to_vector
from task_priority.Tasks.NTkineLib.xacro_to_urdf import convert_xacro_to_urdf


class Robot:

    _config = {"device": "cpu", "dtype": np.float32}

    def __init__(self, device="cpu", dtype=np.float32):
        self.device = device
        self.dtype = dtype

        self.link_names = {}
        self.dof = 0
        self.chain_tasks = []
        Robot._config["device"] = device
        Robot._config["dtype"] = dtype

   
    def xacro_to_urdf(xacro_file_path, output_file_path=None, xacro_arguments=None):
        # os.system(f'xacro {xacro_file} -o {urdf_file}')
        convert_xacro_to_urdf(xacro_file_path, output_file_path, xacro_arguments)

    def add_arm(
        self, urdf_path, start_link: str, ee_link="", tcp_link=None, task_control_name="chain", skip_fixed=True
    ):

        if hasattr(self, task_control_name):
            task_control = getattr(self, task_control_name)
            if not isinstance(task_control, TaskManager):
                raise ValueError("The task_control_name must be a valid chain name")
            task_control.add_arm(urdf_path, start_link, ee_link, tcp_link)
            return

        task_control = TaskManager(dtype=self.dtype, device=self.device, skip_fixed=skip_fixed)
        task_control.add_arm(urdf_path, start_link, ee_link, tcp_link)

        setattr(self, task_control_name, task_control)

    def add_gripper(self, urdf_path, start_link: str, tip_links=[], task_control_name="chain"):
        if hasattr(self, task_control_name):
            task_control = getattr(self, task_control_name)
            if not isinstance(task_control, TaskManager):
                raise ValueError("The task_control_name must be a valid chain name")
            task_control.add_gripper(urdf_path, start_link, tip_links)
            return

        task_control = TaskManager(device=self.device, dtype=self.dtype)
        task_control.add_gripper(urdf_path, start_link, tip_links)
        setattr(self, task_control_name, task_control)

    def set_kinematic_base(
        self,
        urdf_path,
        start_link: str,
        wheels=[],
        wheels_type="omni",
        task_control_name="chain",
        robot_name="arm",
        skip_fixed=True,
    ):
        if hasattr(self, task_control_name):
            task_control = getattr(self, task_control_name)
            if not isinstance(task_control, TaskManager):
                raise ValueError("The task_control_name must be a valid chain name")
            task_control.add_base(urdf_path, start_link, wheels, wheels_type)
            return
        task_control = TaskManager(device=self.device, dtype=self.dtype, skip_fixed=skip_fixed)
        setattr(self, task_control_name, task_control)

    def get_joint_names(self, chain_name="chain"):
        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")
        return chain.get_joint_names()

    def get_joint_limits(self, task_or_urdf="urdf", chain_name="chain"):
        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")
        return chain.get_joint_limits(task_or_urdf)

    def add_task(self, task_name, priority, chain_name="chain"):

        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")
        chain.add_task(task_name, priority)

    def remove_task(self, task_name, chain_name="chain"):
        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")
        # chain.remove_task(task_name) not tested

    def show_status(self, task_name=None, chain_name="chain", **kwargs):

        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")
        chain.show_status(task_name, **kwargs)

    def replace_task(self, task_name, priority, chain_name="chain"):
        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")
        chain.replace_task(task_name, priority)

    def is_in_position(self, chain_name="chain"):
        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")
        return chain.is_in_position()

    def set_override(self, override, chain_name="chain"):
        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")
        chain.set_override(override)

    def set_task_param(self, task_name, chain_name="chain", **kwargs):
        getattr(self, chain_name).set_task_param(task_name, **kwargs)

    def execute(
        self,
        joint_pos=None,
        joint_vel=np.zeros(6),
        force=np.zeros(6),
        odom=np.zeros(7),
        coll_point_cloud=None,
        chain_name="chain",
    ):
        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")

        q_dot = chain.execute(joint_pos, joint_vel, force, odom, coll_point_cloud=coll_point_cloud)
        # print(q_dot)
        # print(chain.get_joint_vel_limit())
        q_dot_clamped = np.clip(q_dot, -chain.joint_vel_limits, chain.joint_vel_limits)

        return q_dot_clamped * (chain.robot_override / 100)

    def get_foward_kinematics(self, joint_pos, chain_name="chain", as_vector=True, joint=-1):
        """
        In:
            joint_pos: torch.tensor, the joint position of the joint_num or joint_name
            chain_name: str, the name of the chain
            as_vector: bool, if True return the foward kinematics as a vector
            as_dict: bool, if True return the foward kinematics as a dictionary
            joint_num: int, the number of the joint or str, the name of the joint
        Out:
            foward: torch.tensor, the foward kinematics of the joint_num or joint_name
        """

        chain = getattr(self, chain_name)
        if not isinstance(chain, TaskManager):
            raise ValueError("The chain_name must be a valid chain name")

        foward = chain.get_foward_kinematics(
            joint_pos, as_dict=(isinstance(joint, str) or joint is None)
        )  # as_dict is a string then returns the foward kinematics as a dictionary

        if as_vector and joint is None:
            if isinstance(foward, dict):
                return {key: homogeneousMatrix_to_vector(value) for key, value in foward.items()}
            else:
                return homogeneousMatrix_to_vector(foward)
        elif as_vector and joint is not None:
            if isinstance(joint, int):
                return homogeneousMatrix_to_vector(foward[joint])
            elif isinstance(joint, str):
                return homogeneousMatrix_to_vector(foward[joint])
