import os, sys
__WS_PATH__ = os.path.abspath(os.path.join(__file__, "../../.."))
sys.path.append(__WS_PATH__) if __WS_PATH__ not in sys.path else None
import numpy as np
from robot_kins.chain_utils.serial_kin import ChainKins
from robot_kins.chain_utils.chain_constructor import ChainConstructor
from robot_components.joint import JointTypes

from robot_components.transforms.transforms import Homogeneus_Matrix_Euler
from robot_kins.chain_utils.urdf_chain_info import ChainInfo



def find_common_link(list_of_chains):
    """
    Trova l'ultimo link comune tra tutte le catene.
    
    list_of_chains: list of Chain - Lista di catene.
        Ogni oggetto Chain ha un attributo `robot_chain` che è una lista di link.
    
    Ritorna:
        common_link_name: str - Nome dell'ultimo link comune.
    
    Solleva:
        ValueError - Se non ci sono link comuni tra tutte le catene.
    """
    # Estrai i set di nomi dei link per ciascuna catena
    chain_links = [{link.name for link in chain} for chain in list_of_chains]

    # Trova l'intersezione tra tutti i set di link
    common_links = set.intersection(*chain_links)
    if not common_links:
        raise ValueError("Nessun link comune trovato tra le catene.")

    # Trova l'ultimo link comune (in base all'ordine nella prima catena)
    for link in reversed(list_of_chains[0]):
        if link.name in common_links:
            return link.name

    raise ValueError("Errore imprevisto: nessun link comune trovato nella prima catena.")

    



class MultipleChainKins(ChainInfo):
    def __init__(self, urdf_file, start_link:str, end_links:list, device='cpu', dtype=np.float32):
        self.device = device
        self.dtype = dtype
        self.start_link = start_link
        self.num_of_fingers = len(end_links)
        list_of_chains = []
        urdf_parser = ChainConstructor(urdf_file)
        
        for end_link in end_links:
            chain = urdf_parser.get_chain(start_link, end_link)
            list_of_chains.append(chain)
            # self.list_of_chains.append(ChainKins(chain, device=device, dtype=dtype))
            # self.list_of_chains[-1].print_chain_info()
            # print(f"Chain from {start_link} to {end_link} created")
            

        common_link = find_common_link(list_of_chains)  
        print(f"Common link: {common_link}")

        main_chain = urdf_parser.get_chain(start_link, common_link)
        main_chain[-1].children = []
        
       
        for end_link in end_links:
            chain = urdf_parser.get_chain(common_link, end_link)
            
            main_chain[-1].children.append(chain)
        
        
        num_tot_links = len(main_chain)
        for child_chain in main_chain[-1].children:
            num_tot_links += len(child_chain) -1
        
        super().__init__(main_chain, device=device, dtype=dtype)
        self.get_info_chain()
        self.forward_tensor = np.zeros((num_tot_links, 4, 4), dtype=dtype)

        link_names = self.get_link_names()
        self.forward_dict = dict.fromkeys(link_names, np.eye(4).astype(dtype))
        
    
    def get_info_chain(self):
        print(f'\033[93m' + f'Path of HAND PALM from {self.start_link} to {self.robot_chain[-1].name}' + '\033[0m')
        self.num_of_fixed_joints = 0
        self.num_of_joints = 0

        for link in self.robot_chain:
            print(f'\033[95m' + f'\tLink name: {link.name} \t\t - Joint name: {link.joint.name} \t\t - Joint type: {link.joint.joint_type} \033[0m')
        # print(f'\033[95m' + f'\tCount of "PALM Fixed" joints: {self.num_of_fixed_joints}  \033[0m')
        # print(f'\033[95m' + f'\tCount of "PALM Not Fixed" joints: {self.num_of_joints} \033[0m')

        children = self.robot_chain[-1].children
        finger_index = 1
        for child_chain in children:
            print(f'\033[93m' + f'Path of FINGER from {self.robot_chain[-1].name} to {child_chain[-1].name}' + '\033[0m')
            for link in child_chain:
                print(f'\033[95m' + f'\tLink name: {link.name} \t\t - Joint name: {link.joint.name} \t\t - Joint type: {link.joint.joint_type} \033[0m')
                
            print(f'\033[95m' + f'\tFinger {finger_index} - Count of Joints: {len(child_chain) - 1} \033[0m')

            finger_index += 1
    
    def get_joint_names(self, skip_fixed=False):
        joint_names = []
        joint_names.extend(super().get_joint_names(skip_fixed))
        for child_chain in self.robot_chain[-1].children:
            for i in range(len(child_chain)-1):
                if not skip_fixed or child_chain[i].joint.joint_type != JointTypes.FIXED:
                    joint_names.append(child_chain[i].joint.name)
        return joint_names
    
    def get_link_names(self, skip_fixed=False):
        link_names = []
        link_names.extend(super().get_link_names(skip_fixed))
        for child_chain in self.robot_chain[-1].children:
            for link in child_chain:
                if not skip_fixed or link.joint.joint_type != JointTypes.FIXED:
                    link_names.append(link.name) if link.name not in link_names else None
        return link_names

    def get_joint_pos_limits(self):
        joint_lower, joint_upper = super().get_joint_pos_limits()
        for child_chain in self.robot_chain[-1].children:
            for i in range(len(child_chain)):
                if  child_chain[i].joint.joint_type != JointTypes.FIXED:
                    joint_lower.append(child_chain[i].joint.lower_lim)
                    joint_upper.append(child_chain[i].joint.upper_lim)
        return joint_lower, joint_upper
    
    def get_joint_vel_limits(self):
        joint_vel = super().get_joint_vel_lim()
        for child_chain in self.robot_chain[-1].children:
            for i in range(len(child_chain)):
                if  child_chain[i].joint.joint_type != JointTypes.FIXED:
                    joint_vel.append(child_chain[i].joint.velocity_lim)
        return joint_vel
    
    def get_manipulator_joint_limits_low(self):
        return self.get_joint_pos_limits()[0]
    
    def get_manipulator_joint_limits_up(self):
        return self.get_joint_pos_limits()[1]
        
    
    def get_link(self, link_name):
        link = super().get_link(link_name)

        if link is not None:
            return link

        for child_chain in self.robot_chain[-1].children:
            for link in child_chain:
                if link.name == link_name:
                    return link
        return None

    def get_link_mesh_path(self, link_name, visual_or_collision='visual'):
        if not visual_or_collision in ['visual', 'collision']:
            raise ValueError("visual_or_collision must be 'visual' or 'collision'")
        

        link = self.get_link(link_name)

        if link is None:
            return None
        
        
        if visual_or_collision == 'visual':
            return link.get_visual_mesh_path()
        elif visual_or_collision == 'collision':
            return link.get_collision_mesh_path()
        
    
    def get_link_mesh_offset(self, link_name, visual_or_collision='visual'):
        if not visual_or_collision in ['visual', 'collision']:
            raise ValueError("visual_or_collision must be 'visual' or 'collision'")
        
        link = self.get_link(link_name)

        if link is None:
            return None
        
        if visual_or_collision == 'visual':
            return link.get_visual_mesh_offset()
        elif visual_or_collision == 'collision':
            return link.get_collision_mesh_offset()

    
    def fk(self, joint_list, base_link_position = None):
        # se base_link_position è None, allora la posizione del primo link è l'origine
        if base_link_position is None:
            self.forward_tensor[0] = Homogeneus_Matrix_Euler(0, 0, 0, 0, 0, 0)
        elif base_link_position.shape == (6):
            self.forward_tensor[0] = Homogeneus_Matrix_Euler(base_link_position[0], base_link_position[1], base_link_position[2], base_link_position[3], base_link_position[4], base_link_position[5])
        elif base_link_position.shape == (4, 4):
            self.forward_tensor[0] = base_link_position
        else:
            raise ValueError("base_link_position must be None or a 6x1 numpy or a 4x4 numpy array")

        j_index = 0

        joint_position = None
        joint_counter = 0
        for i in range(1,len(self.robot_chain)): 
            
            if not self.robot_chain[i-1].joint.joint_type == JointTypes.FIXED and j_index < len(joint_list):
                joint_position = joint_list[j_index]
                j_index += 1
                
            self.forward_tensor[i] = self.forward_tensor[i-1] @ self.robot_chain[i-1].joint.compute_joint_increment(joint_position)
            joint_counter += 1
            # print(joint_counter, self.robot_chain[i-1].joint.name)
           
        start_finger_chain_tensor = self.forward_tensor[joint_counter]
        
        for i in range(len(self.robot_chain[-1].children)):
            joint_counter += 1
            
            if not self.robot_chain[-1].children[i][0].joint.joint_type == JointTypes.FIXED and j_index < len(joint_list):
                joint_position = joint_list[j_index]
                j_index += 1
            self.forward_tensor[joint_counter] = start_finger_chain_tensor  @ self.robot_chain[-1].children[i][0].joint.compute_joint_increment(joint_position)
        
    
    def jacobian_v_joint_point(self, joint_name:str, point_w_coords: np.ndarray=None):
        pass


    def get_forward_as_dict(self, output_type= 'numpy'):
        for i, link_name in enumerate(self.forward_dict.keys()):
            if output_type == 'numpy':
                self.forward_dict[link_name] = self.forward_tensor[i]
            elif output_type == 'torch':
                self.forward_dict[link_name] = torch.tensor(self.forward_tensor[i], device=self.device)
        
        return self.forward_dict



        


if __name__ == "__main__":
    urdf = "test/tiago.urdf"
    start_link = "arm_7_link"
    end_links = ["gripper_left_finger_link", "gripper_right_finger_link"]

    multiple_chain = MultipleChainKins(urdf, start_link, end_links)
    joint_values = [10, 0]
    multiple_chain.fk(joint_values)
    # print(multiple_chain.foward_tensor)
    # print(multiple_chain.get_joint_names())
    # print(multiple_chain.get_link_names())
    # print(multiple_chain.get_joint_pos_limits())
    # print(multiple_chain.get_joint_vel_limits())

    print(multiple_chain.get_link_mesh_path("gripper_link"))
    print(multiple_chain.get_link_mesh_offset("gripper_link"))


    
    


        

        



            