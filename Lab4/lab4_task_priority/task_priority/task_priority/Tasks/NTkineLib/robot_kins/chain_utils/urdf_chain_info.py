import os, sys
sys.path.append(os.path.dirname(__file__))
import numpy as np
from robot_components.joint import JointTypes
from robot_components.transforms.transforms import homogeneousMatrix_to_vector

class ChainInfo:
    
    def __init__(self, chain, device='cpu', dtype=np.float32):

        self.device = device
        self.dtype  = dtype
        self.robot_chain = chain

        self.num_of_fixed_joints  = 0
        self.num_of_joints = 0

        for child in self.robot_chain:
            if child.joint.joint_type == JointTypes.FIXED:
                self.num_of_fixed_joints += 1
            else:
                self.num_of_joints += 1
        
        self.__init_foward()
        self.__init_jacobian()
        self.forward_dict = dict.fromkeys(self.get_link_names(), None)


    def print_chain_info(self):
        print(f'\033[93m' + f'[INFO]Path you choose for FOWARD KINEMATIC:" \033[0m')
        print(f'\033[95m' + f'\tCount of "Fixed" joints: {self.num_of_fixed_joints}  \033[0m')
        print(f'\033[95m' + f'\tCount of "Not Fixed" joints: {self.num_of_joints} \033[0m')
        for i in range(len(self.robot_chain)):
            print(f'\033[95m' + f'\tLink name: {self.robot_chain[i].link.name} \t\t\t - Joint name: {self.robot_chain[i].joint.name} \t\t - Joint type: {self.robot_chain[i].joint.joint_type} \033[0m')
            

    def __init_foward(self):
        
        self.forward_tensor = np.ones((len(self.robot_chain), 4, 4), dtype=self.dtype)
        self._first_link_tf_ = np.eye(4).astype(self.dtype)
        self.forward_tensor[0] = self._first_link_tf_

    def __init_jacobian(self):
        self.jacobian_matrix = np.zeros((6, self.num_of_joints), dtype=self.dtype)

    def get_num_of_fixed_joints(self):
        return self.num_of_fixed_joints
    
    def get_num_of_joints(self):
        return self.num_of_joints

    def get_joint_names(self, skip_fixed=False):
        joint_names = []
        for i in range(len(self.robot_chain)-1):
            if not skip_fixed or self.robot_chain[i].joint.joint_type != JointTypes.FIXED:
                joint_names.append(self.robot_chain[i].joint.name)
        return joint_names
    
    def get_link_names(self, skip_fixed=False):
        link_names = []

        for i in range(len(self.robot_chain)):
            if not skip_fixed or self.robot_chain[i].joint.joint_type != JointTypes.FIXED:
                link_names.append(self.robot_chain[i].link.name)

        return link_names

    def map_link_joint_type(self):
        link_joint_map = {}
        
        for i in range(len(self.robot_chain)):
            joint_type_map = {}
            joint_type_map[self.robot_chain[i].joint.name] = self.robot_chain[i].joint.get_type()
            link_joint_map[self.robot_chain[i].link.name] = joint_type_map
            
        return link_joint_map

    def get_link(self, link_name):
        for i in range(len(self.robot_chain)):
            if self.robot_chain[i].name == link_name:
                return self.robot_chain[i]
        return None

    def get_joint_pos_limits(self):
        limit_upper = []
        limit_lower = []

        for i in range(len(self.robot_chain)):
            if self.robot_chain[i].joint.joint_type != JointTypes.FIXED:
                limit_upper.append(self.robot_chain[i].joint.upper_lim)
                limit_lower.append(self.robot_chain[i].joint.lower_lim)

        return limit_lower, limit_upper

    def get_joint_vel_lim(self):
        max_vel = []

        for i in range(len(self.robot_chain)):
            if self.robot_chain[i].joint.joint_type != JointTypes.FIXED:
                max_vel.append(self.robot_chain[i].joint.velocity_lim)
        
        return max_vel
    
    def get_manipulator_joint_limits_low(self):
        return self.get_joint_pos_limits()[0]
    
    def get_manipulator_joint_limits_upper(self):
        return self.get_joint_pos_limits()[1]

    def get_link_mesh_path(self, link_name, visual_or_collision='visual'):
        if not visual_or_collision in ['visual', 'collision']:
            raise ValueError("Input Var must be 'visual' or 'collision'")
        
        link = self.get_link(link_name)

        if link is None:
            return None

        if visual_or_collision == 'visual':
            return link.get_visual_mesh_path()
        elif visual_or_collision == 'collision':
            return link.get_collision_mesh_path()

    
    def get_link_mesh_offset(self, link_name, visual_or_collision='visual'):
        if not visual_or_collision in ['visual', 'collision']:
            raise ValueError("Input Var must be 'visual' or 'collision'")

        
        link = self.get_link(link_name)
        if link is None:
            return None

        if visual_or_collision == 'visual':
            return link.get_visual_mesh_offset()
        elif visual_or_collision == 'collision':
            return link.get_collision_mesh_offset()


    def get_ee_vector(self):
        return homogeneousMatrix_to_vector(self.forward_tensor[-1])
