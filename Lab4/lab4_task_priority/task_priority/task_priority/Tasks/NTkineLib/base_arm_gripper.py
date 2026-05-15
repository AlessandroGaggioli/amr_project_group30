import os, sys
sys.path.append(os.path.dirname(__file__))
import numpy as np
from robot_kins.serial_manipulator import Manipulator
from robot_kins.gripper import ParallelGripper


class RobotDynKins:
    def __init__(self, device='cpu', dtype=np.float32):
        self.device = device
        self.dtype = dtype

        self.joint_limits_upper =[]
        self.joint_limits_low = []
        self.joint_vel_lim = []
        self.joint_names = []
        self.link_names = []
        self.num_of_joints = 0
        self.num_of_fixed_joints = 0
        self.num_of_links = 0

        self.base_name = 'base'
        self.arm_name = 'arm'
        self.gripper_name = 'gripper'


        self.robot_cache = {}

        self.start_link = None
        
    def add_base(self, urdf_file, start_link, wheel_names:list):
        self.robot_cache[self.base_name] = None
        pass


    def add_arm(self, urdf_file:str, start_link:str, ee_link:str, tcp_link:str=None):
        self.robot_cache[self.arm_name] = Manipulator(urdf_file, start_link, ee_link, tcp_link, device=self.device, dtype=self.dtype)
        self.__update_parametrs__(self.arm_name)

    def add_gripper(self, urdf_file:str, start_link:str, end_link:list):
        self.robot_cache[self.gripper_name] = ParallelGripper(urdf_file, start_link, end_link, device=self.device, dtype=self.dtype)
        self.__update_parametrs__(self.gripper_name)

    def jacobian_stack(self):
        jacobian = np.empty((6, 0), dtype=self.dtype)
        for robot_name in self.robot_cache.keys(): #aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaahhhhhhh schifooooo
            jacobian = np.hstack((jacobian, self.robot_cache[robot_name].jacobian_matrix))
        return jacobian
            
    def compute(self, joint_list):
        start_count = 0
        end_count = 0
        base_pos = np.eye(4, dtype=self.dtype)
        if self.base_name in self.robot_cache.keys():
            end_count += self.robot_cache[self.base_name].get_num_of_joints()
            self.robot_cache[self.base_name].compute(joint_list)
            
        
        if self.arm_name in self.robot_cache.keys():
            end_count += self.robot_cache[self.arm_name].get_num_of_joints()
            self.robot_cache[self.arm_name].compute(joint_list[start_count:end_count], base_pos)
            start_count = end_count
            base_pos = self.robot_cache[self.arm_name].get_ee_matrix()
        

        if self.gripper_name in self.robot_cache.keys():
            end_count += self.robot_cache[self.gripper_name].get_num_of_joints()
            self.robot_cache[self.gripper_name].compute(joint_list[start_count:end_count], base_pos)
            start_count = end_count
    
    def __update_parametrs__(self, robot_name):
        self.joint_limits_upper += self.robot_cache[robot_name].get_manipulator_joint_limits_upper()
        self.joint_limits_low += self.robot_cache[robot_name].get_manipulator_joint_limits_low()
        self.joint_vel_lim += self.robot_cache[robot_name].get_joint_vel_lim()
        self.joint_names += self.robot_cache[robot_name].get_joint_names()
        self.link_names += self.robot_cache[robot_name].get_link_names()
        self.num_of_joints += self.robot_cache[robot_name].get_num_of_joints()
        self.num_of_fixed_joints += self.robot_cache[robot_name].get_num_of_fixed_joints()
        
        print(self.joint_limits_upper)
        print(self.joint_limits_low)
        print(self.joint_vel_lim)
        print(self.joint_names)
        print(self.link_names)
        print(self.num_of_joints)
        print(self.num_of_fixed_joints)
        print('-------------------')


    
    def get_ee_vector(self):
        return self.robot_cache['arm'].get_ee_vector()

    def get_foward_tensor(self, index = -1, robot_name=None):
        
        return self.robot_cache[robot_name].forward_tensor[index]
        
    def get_num_of_fixed_joints(self, robot_name):
        return self.robot_cache[robot_name].get_num_of_fixed_joints()
    
    def get_num_of_joints(self, robot_name):
        return self.robot_cache[robot_name].get_num_of_joints()
    
    def get_joint_names(self, robot_name, skip_fixed=False):
        joint_names = []
        for robot_name in self.robot_cache.keys():
            joint_names += self.robot_cache[robot_name].get_joint_names(skip_fixed)
        return joint_names

    
    def get_link_names(self):
        link_names = []
        for robot_name in self.robot_cache.keys():
            link_names += self.robot_cache[robot_name].get_link_names()
        return link_names


    def get_forward_as_dict(self, output_type= 'numpy'):
        fow_dict = {}
        for robot_name in self.robot_cache.keys():
            fow_dict.update(self.robot_cache[robot_name].get_forward_as_dict(output_type=output_type))
        
        return fow_dict

    def jacobian_v_joint_point(self, joint_name:str, point_w_coords: np.ndarray=None, robot_name=None):
        if robot_name is not None:
            return self.robot_cache[robot_name].jacobian_v_joint_point(joint_name, point_w_coords)

        for robot_name in self.robot_cache.keys():
            jacobian = self.robot_cache[robot_name].jacobian_v_joint_point(joint_name, point_w_coords)
            if jacobian is not None:
                return jacobian
            
    def map_link_joint_type(self, robot_name='manipulator'):
        return self.robot_cache[robot_name].map_link_joint_type()
    
    def get_manipulator_joint_limits_low(self, robot_name=None):
        if robot_name is not None:
            return self.robot_cache[robot_name].get_manipulator_joint_limits_low()
        
        joint_limits = []
        for robot_name in self.robot_cache.keys():
            joint_limits += self.robot_cache[robot_name].get_manipulator_joint_limits_low()
        
        return joint_limits

    def get_manipulator_joint_limits_upper(self, robot_name=None):
        if robot_name is not None:
            return self.robot_cache[robot_name].get_manipulator_joint_limits_upper()
        
        joint_limits = []
        for robot_name in self.robot_cache.keys():
            joint_limits += self.robot_cache[robot_name].get_manipulator_joint_limits_upper()
        
        return joint_limits
    
    def get_joint_vel_lim(self, robot_name=None):
        if robot_name is not None:
            return self.robot_cache[robot_name].get_joint_vel_lim()
        
        joint_limits = []
        for robot_name in self.robot_cache.keys():
            joint_limits += self.robot_cache[robot_name].get_joint_vel_lim()
        
        return joint_limits
    
    def get_link_mesh_path(self, link_name, visual_or_collision, robot_name=None):
        if robot_name is not None:
            return self.robot_cache[robot_name].get_link_mesh_path(link_name, visual_or_collision)

        for robot_name in self.robot_cache.keys():
            mesh_path = self.robot_cache[robot_name].get_link_mesh_path(link_name, visual_or_collision)
            if mesh_path is not None:
                return mesh_path
    
    def get_link_mesh_offset(self, link_name, visual_or_collision, robot_name=None):
        if robot_name is not None:
            return self.robot_cache[robot_name].get_link_mesh_offset(link_name, visual_or_collision)

        for robot_name in self.robot_cache.keys():
            mesh_offset = self.robot_cache[robot_name].get_link_mesh_offset(link_name, visual_or_collision)
            if mesh_offset is not None:
                return mesh_offset        
    
    def gravity_compensation(self, joint_positions, robot_name='manipulator'):
        return self.robot_cache[robot_name].gravity_compensation(joint_positions)
    
    def get_robot_names(self):
        return list(self.robot_cache.keys())





if __name__ == "__main__":
    from test.test_kdl import compute_jacobian, compute_gravity_compensation, get_link_masses
    path_to_file = os.path.dirname(__file__)
    path_to_urdf = os.path.join(path_to_file, 'test', 'panda.urdf')

    chain = RobotDynKins()
    start_link = 'world'
    end_link = 'panda_link8'
    urdf = path_to_urdf

    joint_positions = [.1]*8

    chain.add_arm(urdf, start_link, end_link)
    chain.add_gripper(urdf, end_link, ['panda_leftfinger', 'panda_rightfinger'])
    chain.compute(joint_positions)

    print(chain.get_forward_as_dict(output_type='numpy'))

