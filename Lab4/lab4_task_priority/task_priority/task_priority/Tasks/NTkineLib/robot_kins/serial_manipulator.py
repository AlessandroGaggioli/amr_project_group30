import os, sys
__WS_PATH__ = os.path.abspath(os.path.join(__file__, "../.."))
sys.path.append(__WS_PATH__) if __WS_PATH__ not in sys.path else None

from robot_kins.chain_utils.serial_kin import ChainKins
from robot_kins.chain_utils.chain_constructor import ChainConstructor
import numpy as np
from robot_kins.chain_utils.robot_components.transforms.transforms import homogeneousMatrix_to_vector


class Manipulator(ChainKins):
    def __init__(self, urdf_file, start_link, end_effector, tcp, device='cpu', dtype=np.float32):
        urdf_parser = ChainConstructor(urdf_file)
        tcp = end_effector if tcp is None else tcp
        
        chain = urdf_parser.get_chain(start_link, tcp)

        super().__init__(chain, device=device, dtype=dtype)

        super().print_chain_info()
        
        self.robot_name = 'manipulator'
        
        self.tcp_index = self.get_fk_index(tcp)
        self.ee_index = self.get_fk_index(end_effector) 
            

    
    def get_fk_index(self, link_name): #
        if link_name in self.get_link_names():
            return self.get_link_names().index(link_name)
        else:
            raise ValueError(f"Link {link_name} not found in the robot chain.")
        
    def get_ee_vector(self):
        return homogeneousMatrix_to_vector(self.forward_tensor[self.ee_index])
    
    def get_tcp_vector(self):
        return homogeneousMatrix_to_vector(self.forward_tensor[self.tcp_index])
    
    def get_ee_matrix(self):
        return self.forward_tensor[self.ee_index]
    
    def get_tcp_matrix(self):
        return self.forward_tensor[self.tcp_index]

    
    # def add_gripper(self, urdf_file, attached_to, fingers, gripper_type='parallel'):
        
    #     if not attached_to in self.get_link_names():
    #         raise ValueError(f"Link {attached_to} not found in the robot chain.")

    #     if gripper_type == 'parallel':
    #         finger_1_name, finger_2_name = fingers
    #     else:
    #         raise ValueError(f"Gripper type {gripper_type} not supported.")
        
    #     self.gripper = ParallelGripper(urdf_file, attached_to, finger_1_name, finger_2_name, device=self.device, dtype=self.dtype)
        
    
    
    # def compute(self, joint_list):
    #     start_count = 0
    #     end_count = self.no_fix_joints
    #     super().compute(joint_list[start_count:end_count])
    #     if hasattr(self, 'gripper'):
    #         self.gripper.compute(joint_list[-self.gripper.num_of_joints], self.forward_tensor[-1])

        
            
if __name__ == "__main__":
    urdf = "test/panda.urdf"
    start_link = "world"
    end_link = "panda_link8"
    manipulator = Manipulator(urdf, start_link, end_link)
    joint_list = [0, 0, 0, 0, 0, 0, 0, 0]
    manipulator.compute(joint_list)
    # print(manipulator.get_joint_names())
    # print(manipulator.get_link_names())
    print(manipulator.get_joint_pos_limits())
    # manipulator.get_manipulator_fk(joint_list, as_dict=True)
