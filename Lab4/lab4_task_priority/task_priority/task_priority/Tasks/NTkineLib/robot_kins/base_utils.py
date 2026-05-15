from .robot_components.urdf_unpack import _filter_child_map, _component_constructor
from ..transforms.transforms import rotation_matrix_to_euler_angles
from collections import OrderedDict
import numpy as np
from .robot_components.joint import JointTypes
from ..transforms.rotations import quaternion_to_euler

class WheelsTypes:
    OMNI = 1
    DIFFERENTIAL = 2

    @staticmethod
    def get_corr_enum(wheel_type):
        if wheel_type == 'omni':
            return WheelsTypes.OMNI
        elif wheel_type == 'differential':
            return WheelsTypes.DIFFERENTIAL
        else:
            raise RuntimeError("wheel specified as {} type not, but we only support {}".format(wheel_type, ['omni', 'differential']))


class BaseKinematics():
    def __init__(self, robot_xml, start_link, wheels =[], wheel_type='omni',  device='cpu', dtype=np.float32, skip_fixed=False):
        self.device          = device
        self.dtype           = dtype
        self.wheels          = wheels
        self.wheels_type     = WheelsTypes.get_corr_enum(wheel_type)
        self.start_link      = start_link
        self.h_matrix        = np.zeros((len(self.wheels),3), dtype=dtype)
        self.gamma           = np.array(np.pi/4, dtype=dtype)
        self.r               = np.array(1/0.127, dtype=dtype)            #questo dovrebbe diventare un valore da passare
        self.base_components = None
        merged_dict          = OrderedDict()
        filtered_child       = []
        self.jacobian_matrix = np.array([[1, 0, 0], [0, 1, 0,],[0, 0, 0,],[0, 0, 0,],[0, 0, 0,],[0,0,1]], dtype=dtype)
 
        for wheel in wheels:
            wheel_chain = robot_xml.get_chain(start_link, wheel)
            filtered_child.append(_filter_child_map(robot_xml, wheel_chain))
      
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

        self.base_components = _component_constructor(robot_xml,  merged_dict, device=device, dtype=dtype)
        self.robot_chain     = self.base_components[0].get_chain(skip_fixed=skip_fixed)
        self.jacobian_chain  = self.base_components[0].collapse_fixed_joints()

        self.fixed_joints  = 0
        self.no_fix_joints = 0
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
        self._identity_matrix_ = np.eye(4, dtype=self.dtype)
        
    def get_num_of_joints(self, skip_fixed=True):
        if skip_fixed:
            return self.no_fix_joints  
        else:
            return self.fixed_joints + self.no_fix_joints

    # def to(self, device='cpu', dtype=torch.float32):
    #     self.device = device
    #     self.dtype = dtype

    def to_torch(self):
        self.forward_tensor = torch.from_numpy(self.forward_tensor).to(device=self.device, dtype=self.dtype)
        self.jacobian_matrix = torch.from_numpy(self.jacobian_matrix).to(device=self.device, dtype=self.dtype)


    def to_numpy(self):
        self.forward_tensor = self.forward_tensor.cpu().detach().numpy()
        self.jacobian_matrix = self.jacobian_matrix.cpu().detach().numpy()

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

    def compute_hi(self, joint_angles: torch.Tensor):
  
        self._fk(joint_angles)

        for i in range(len(self.wheels)):
            self.h_matrix[i,0:3] = self._compute_hi(i)
        return self.h_matrix
    
    def _compute_hi(self, i):
        
        dn = self.forward_tensor[i][-1, 0:3, 3:4]
        xyb_base = rotation_matrix_to_euler_angles(self.forward_tensor[i][-1, 0:3, 0:3])

        if self.wheels_type == WheelsTypes.OMNI and i in range(0, len(self.wheels), 3):

            hi_matrix = self.r * np.array([1, np.tan(self.gamma)]) @ np.array([[np.cos(xyb_base[2]), np.sin(xyb_base[2])], [-np.sin(xyb_base[2]), np.cos(xyb_base[2])]]) @ np.array([[-dn[1][0], 1, 0 ], [dn[0][0], 0, 1]])
        elif self.wheels_type == WheelsTypes.OMNI and i in range(1, len(self.wheels)-1):
            hi_matrix = self.r * np.array([1, np.tan(-self.gamma)]) @ np.array([[np.cos(xyb_base[2]), np.sin(xyb_base[2])], [-np.sin(xyb_base[2]), np.cos(xyb_base[2])]]) @ np.array([[-dn[1][0], 1, 0 ], [dn[0][0], 0, 1]])

        return np.array([[hi_matrix[0],hi_matrix[1], hi_matrix[2]]])
    
    def compute_vel_base(self, h_matrix, odom_angles, joint_velocities):

        odom_euler = quaternion_to_euler(odom_angles[3], odom_angles[4], odom_angles[5], odom_angles[6])

        c = np.cos(odom_euler[2]).astype(dtype=self.dtype)
        s = np.sin(odom_euler[2]).astype(dtype=self.dtype)

        phyxy_vel = np.linalg.pinv(h_matrix @ np.array([[1,0,0],[0, c, s], [0, -s, c]])) @ np.array(joint_velocities).reshape(4,1)
        xyphy_vel = np.array([[phyxy_vel[1], phyxy_vel[2], phyxy_vel[0]]])  #torch.tensor([phyxy_vel[1], phyxy_vel[2], phyxy_vel[0]])
        return (xyphy_vel).reshape(1, 3)