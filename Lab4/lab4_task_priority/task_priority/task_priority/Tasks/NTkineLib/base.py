import os, sys
__CURRENT_DIR__ = os.path.dirname(os.path.abspath(__file__))
__PARENT_DIR__ = os.path.dirname(__CURRENT_DIR__)
sys.path.append(__PARENT_DIR__) if __PARENT_DIR__ not in sys.path else None
sys.path.append(__CURRENT_DIR__) if __CURRENT_DIR__ not in sys.path else None
from urdf_utils.urdf_parser_py.urdf import URDF
from robot_kins.kin_utils.robot_components.frame import Component
from robot_kins.kin_utils.robot_components.joint import Joint
from robot_kins.kin_utils.robot_components.link import Link
from robot_kins.kin_utils.foward import FowardKinematics
from robot_kins.kin_utils.jacobian import Jacobian
from robot_kins.kin_utils.chain_kins import ManipulatorKinematics
from Tasks.KdlTorch_Kins.kins_utils.chain_kin.base_utils import BaseKinematics
import torch, numpy as np
import time
from robot_kins.transforms.transforms import Homogeneus_Matrix_Quaternion, rotation_matrix_to_euler_angles
from robot_kins.transforms.rotations import quaternion_to_euler
from collections import OrderedDict
from robot_kins.kin_utils.base_utils import WheelsTypes
from robot_kins.kin_utils.robot_components.joint import JointTypes
from robot_kins.kin_utils.robot_components.urdf_unpack import _filter_child_map, _component_constructor


def create_chain_component(class_name):
    # Creazione della classe utilizzando type() e ereditando dalla classe template
    return type(class_name, (Component,), {})


class Base():
    def __init__(self, urdf_file, start_link='', wheels=[],wheels_type='omni', device='cpu', dtype=np.float32, skip_fixed=True):
        self.dtype           = dtype
        self.wheels          = wheels
        self.wheels_type     = WheelsTypes.get_corr_enum(wheels_type)
        self.start_link      = start_link
        self.h_matrix        = np.zeros((len(self.wheels),3), dtype=dtype)
        self.gamma           = np.array(np.pi/4, dtype=dtype)
        self.r               = np.array(1/0.127, dtype=dtype)            #questo dovrebbe diventare un valore da passare
        self.base_components = None
        self.robot_chain     = []
        self.fixed_joints    = 0
        self.no_fix_joints   = 0
        self.joint_vel       = np.empty(0, dtype=dtype)

        self.forward_tensor    = np.eye(4, dtype=self.dtype)
        self._identity_matrix_ = np.eye(4, dtype=self.dtype)

        self.odom_matrix       = np.eye(4, dtype=self.dtype)
        self.jacobian_matrix   = np.empty((6, 0), dtype=self.dtype)

        if len(wheels) != 0:
            
            robot = URDF.from_xml_file(urdf_file)
            
            merged_dict          = OrderedDict()
            filtered_child       = []
            self.jacobian_matrix = np.array([[1, 0, 0], [0, 1, 0,],[0, 0, 0,],[0, 0, 0,],[0, 0, 0,],[0,0,1]], dtype=dtype)
    
            for wheel in wheels:
                wheel_chain = robot.get_chain(start_link, wheel)
                filtered_child.append(_filter_child_map(robot, wheel_chain))
        
            def merge_ordereddicts(dicts):
                merged_dict = OrderedDict()
                for d in dicts:
                    for key, associations in d.items():
                        if key not in merged_dict:
                            # Aggiungi la chiave e le associazioni
                            merged_dict[key] = list(associations)
                        else:
                            # Unire le associazioni preservando l'ordine ed evitando duplicati
                            existing_associations = merged_dict[key]
                            for item in associations:
                                if item not in existing_associations:
                                    existing_associations.append(item)
                            merged_dict[key] = existing_associations
                return merged_dict

            # Applicazione della funzione
            merged_dict = merge_ordereddicts(filtered_child)

            self.base_components = _component_constructor(robot,  merged_dict, device=device, dtype=dtype)
            self.robot_chain     = self.base_components[0].get_chain(skip_fixed=skip_fixed)
            self.jacobian_chain  = self.base_components[0].collapse_fixed_joints()
   
            joint_type_list    = []
            joint_names        = []
            link_names         = []
        
            for child in self.robot_chain:
                if child.joint.joint_type == JointTypes.FIXED:
                    self.fixed_joints += 1
                else:
                    self.no_fix_joints += 1 
                joint_type_list.append(child.joint.joint_type)
                joint_names.append(child.joint.name)
                link_names.append(child.link.name)
        
            print(f'\033[93m' + f'[INFO]Path you choose for FOWARD KINEMATIC:" \033[0m')
            print(f'\033[95m' + f'\tCount of "Fixed" joints: {self.fixed_joints}  \033[0m')
            print(f'\033[95m' + f'\tCount of "Not Fixed" joints: {self.no_fix_joints} \033[0m')

            for i in range(len(joint_type_list)):
                print(f'\033[95m' + f'\tJoint name: {joint_names[i]} \t\t - Joint type: {joint_type_list[i]} \033[0m')

            self.num_foward_tensor = self.fixed_joints + self.no_fix_joints +1 - (len(wheels)-1)
            self._identity_tensor_ = np.tile(np.eye(4, dtype=self.dtype), (self.num_foward_tensor, 1, 1))
            self.forward_tensor    = np.stack([self._identity_tensor_] * len(wheels), axis=0)

            if device == 'cuda' and torch.cuda.is_available():
                self.device = torch.device('cuda')
            else:
                self.device = torch.device('cpu')
            
            print("\033[92m" + 'Set robot chain to {}'.format(self.device) + "\033[0m")

            print(f'\033[92m' + f'Loading urdf file: {urdf_file} \033[0m')
        
    
    def to(self, device='cpu', dtype=torch.float32): #test this
        self.device = device
        self.dtype = dtype
    
    @staticmethod
    def to_tensor(data, device='cpu', dtype=torch.float32): #new
        if isinstance(data, list):
            data = torch.tensor(data, device=device, dtype=dtype)
        elif isinstance(data, np.ndarray):
            data = torch.from_numpy(data).to(device=device, dtype=dtype)
        else:
            raise ValueError('Data must be a list or a numpy array')
        
        return data
    
    def to_torch(self):
        self.forward_tensor = torch.from_numpy(self.forward_tensor).to(device=self.device, dtype=self.dtype)
        self.jacobian_matrix = torch.from_numpy(self.jacobian_matrix).to(device=self.device, dtype=self.dtype)


    def to_numpy(self):
        self.forward_tensor = self.forward_tensor.cpu().detach().numpy()
        self.jacobian_matrix = self.jacobian_matrix.cpu().detach().numpy()
    
    def get_num_of_joints(self, skip_fixed=True):
        if skip_fixed:
            if(self.no_fix_joints - 1 < 0):
                return 0
            else: return self.no_fix_joints - 1
        else:
            if(self.fixed_joints + self.no_fix_joints - 1 < 0):
                return 0
            else: return self.fixed_joints + self.no_fix_joints - 1
        
    def get_joint_names(self):
        return []
    
    def get_link_names(self):
        return []


    def _fk(self, joint_angles):
        j_index             = 0
        child_index         = len(self.robot_chain) - len(self.wheels)
        joint_position      = None # if joint is fixed, it will not be used, but if it is not fixed, it will be used to compute the joint increment
        self.forward_tensor = np.stack([self._identity_tensor_] * len(self.wheels), axis=0)

        # Imposta la prima trasformazione di ogni tensore come la matrice identità iniziale
        self.forward_tensor[:, 0] = self._identity_matrix_

        #Da verificare se ci possano essere  altri tipi di joint oltre a fissi e ruote 
        for i in range(len(self.forward_tensor)):

            for j in range(self.num_foward_tensor-1): 
                if self.robot_chain[j].joint.joint_type != JointTypes.FIXED and j == self.num_foward_tensor-2:
                    joint_position = joint_angles[j_index]
                    self.forward_tensor[i][j+1] = self.forward_tensor[i][j] @ self.robot_chain[child_index].joint.compute_joint_increment(joint_position)
                    j_index += 1
                    child_index += 1   
                else:
                    self.forward_tensor[i][j+1] = self.forward_tensor[i][j] @ self.robot_chain[j].joint.compute_joint_increment(joint_position) 

    def compute_hi(self, joint_angles: np.array):
  
        self._fk(joint_angles)

        for i in range(len(self.wheels)):
            dn = self.forward_tensor[i][-1, 0:3, 3:4]
            xyb_base = rotation_matrix_to_euler_angles(self.forward_tensor[i][-1, 0:3, 0:3])

            if self.wheels_type == WheelsTypes.OMNI and i in range(0, len(self.wheels), 3):

                hi_matrix = self.r * np.array([1, np.tan(self.gamma)]) @ np.array([[np.cos(xyb_base[2]), np.sin(xyb_base[2])], [-np.sin(xyb_base[2]), np.cos(xyb_base[2])]]) @ np.array([[-dn[1][0], 1, 0 ], [dn[0][0], 0, 1]])
            elif self.wheels_type == WheelsTypes.OMNI and i in range(1, len(self.wheels)-1):
                hi_matrix = self.r * np.array([1, np.tan(-self.gamma)]) @ np.array([[np.cos(xyb_base[2]), np.sin(xyb_base[2])], [-np.sin(xyb_base[2]), np.cos(xyb_base[2])]]) @ np.array([[-dn[1][0], 1, 0 ], [dn[0][0], 0, 1]])

            self.h_matrix[i,0:3] = np.array([[hi_matrix[0],hi_matrix[1], hi_matrix[2]]])
        
  
    
    def compute(self, odom = np.zeros(7), joint_positions = np.zeros(4), joint_velocities= np.zeros(4)) -> np.array:
        '''
            Calcola la cinematica diretta e il jacobiano del manipolatore

            Args:
                joint_positions (torch.Tensor): Tensor di dimensione (n,) contenente le posizioni delle articolazioni del manipolatore.

            Returns:
                tuple: Tupla contenente la cinematica diretta e il jacobiano del manipolatore.
        '''
        
        # if isinstance(joint_positions, list) or isinstance(joint_positions, np.ndarray):   
        #     raise ValueError('Joint positions must be a tensor, call to_tensor() method')
        if np.mean(odom) == 0:
            return
        self.odom_matrix = Homogeneus_Matrix_Quaternion(odom[0], odom[1], odom[2], odom[6], odom[3], odom[4],odom[5])
        self.compute_hi(joint_positions)

        odom_euler = quaternion_to_euler(odom[3], odom[4], odom[5], odom[6])

        c = np.cos(odom_euler[2]).astype(dtype=self.dtype)
        s = np.sin(odom_euler[2]).astype(dtype=self.dtype)

        phyxy_vel = np.linalg.pinv(self.h_matrix @ np.array([[1,0,0],[0, c, s], [0, -s, c]])) @ np.array(joint_velocities).reshape(4,1)
        self.joint_vel = np.array([[phyxy_vel[1], phyxy_vel[2], phyxy_vel[0]]]).reshape(1, 3).flatten() 

        #self.joint_vel = np.concatenate((xyphy_vel, joint_velocities[self.get_num_of_joints():][np.newaxis]), axis=1).flatten()

    def get_jacobian(self):
        return self.jacobian_matrix
    
    def get_joint_vel_base_manipulator(self):
        return self.joint_vel
       
    def get_odom_matrix(self):
        return self.odom_matrix
        