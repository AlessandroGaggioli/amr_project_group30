
import numpy as np
# from transforms.transforms import homogeneousMatrix_to_vector
from robot_kins.chain_utils.urdf_chain_info import ChainInfo
from robot_kins.chain_utils.chain_constructor import ChainConstructor
from robot_kins.chain_utils.robot_components.joint import JointTypes


import logging

# Configura il logger per visualizzare i messaggi in modo chiaro
logging.basicConfig(
    level=logging.WARNING,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
# Emetti il messaggio di avviso
logging.warning(
    "Attenzione: Prima di chiamare i metodi per ottenere le matrici, \n"
    "assicurati di eseguire il metodo 'compute()' sulla classe 'KinematicsBase'. \n"
    "Questo garantisce che i calcoli siano aggiornati e corretti. \n",
)

GRAVITY_VECTOR = np.array([0, 0, -9.81])

class ChainKins(ChainInfo):
    def __init__(self, chain, device='cpu', dtype=np.float32):
        super().__init__(chain, device=device, dtype=dtype)
    
    def _fk(self, joint_list):
        j_index = 0

        joint_position = None # if joint is fixed, it will not be used, but if it is not fixed, it will be used to compute the joint increment
        # self.forward_tensor= self._identity_tensor_

        self.forward_tensor[0] = self._first_link_tf_

        for i in range(1,len(self.robot_chain)): 
                        
            if not self.robot_chain[i-1].joint.joint_type == JointTypes.FIXED and j_index < len(joint_list):
                joint_position = joint_list[j_index]
                j_index += 1
            self.forward_tensor[i] = self.forward_tensor[i-1] @ self.robot_chain[i-1].joint.compute_joint_increment(joint_position)
            
        # print("Forward Tensor", self.forward_tensor)
        
    def jacobian(self):
        j_index = 0
        dn = self.forward_tensor[-1, 0:3, 3:4]
        for i in range(len(self.robot_chain)):
            if self.robot_chain[i].joint.joint_type != JointTypes.FIXED:
                rotation_matrix = self.forward_tensor[i + 1, 0:3, 0:3]
                joint_axis = self.robot_chain[i].joint.axis.reshape(3, 1)
                
                if self.robot_chain[i].joint.joint_type == JointTypes.REVOLUTE:
                    self.jacobian_matrix[0:3, j_index:j_index + 1] = np.cross(np.dot(rotation_matrix, joint_axis.flatten()), (dn - self.forward_tensor[i + 1][0:3, 3:4]).flatten()).reshape(3, 1)
                    self.jacobian_matrix[3:6, j_index:j_index + 1] = np.dot(rotation_matrix, joint_axis)
                elif self.robot_chain[i].joint.joint_type == JointTypes.PRISMATIC:
                    self.jacobian_matrix[0:3, j_index:j_index + 1] = np.matmul(rotation_matrix, joint_axis)
                    self.jacobian_matrix[3:6, j_index:j_index + 1] = np.zeros((3, 1), dtype=self.dtype)
                j_index += 1
    

    def compute(self, joint_list, first_link_tf=None):
        if first_link_tf is not None:
            self._first_link_tf_ = first_link_tf
        self._fk(joint_list)
        self.jacobian()
        

    def get_manipulator_fk(self, joint_positions : np.array, as_dict=False):
        
        self._fk(joint_positions)

        if as_dict:
            return self.get_forward_as_dict()
        else:
            return self.forward_tensor

    def jacobian_v_joint_point(self, joint_name:str, point_w_coords: np.ndarray=None):
        # Create a mapping from joint names to their indices
        joint_index_map = {joint.name: i for i, joint in enumerate(self.robot_chain)}
        # print(joint_index_map)
 
 
        # Check if the provided joint_name exists in the mapping
        if joint_name not in joint_index_map:
            return
            raise ValueError(f"Joint '{joint_name}' not found in the robot chain.")
 
        # Get the index of the joint using its name
        joint_i = joint_index_map[joint_name]

 
        # Calculate dn: the position of the point in the joint's coordinates
        jacobian_matrix = np.zeros((3, self.num_of_joints), dtype=self.dtype)


        
        if point_w_coords is None:
            point_w_coords = self.forward_tensor[joint_i, 0:3, 3:4].flatten()

        j_index = 0
 
        for i in range(joint_i+1): # +1 to include the joint itself
            if self.robot_chain[i].joint.joint_type != JointTypes.FIXED:
                rotation_matrix = self.forward_tensor[i + 1, 0:3, 0:3]
                joint_axis = self.robot_chain[i].joint.axis.reshape(3, 1)           
                if self.robot_chain[i].joint.joint_type == JointTypes.REVOLUTE:
                    jacobian_matrix[0:3, j_index:j_index + 1] = np.cross(np.dot(rotation_matrix, joint_axis.flatten()), (point_w_coords - np.squeeze(np.reshape(self.forward_tensor[i + 1][0:3, 3:4], (1, 3)), axis=0))).reshape(3, 1)
                elif self.robot_chain[i].joint.joint_type == JointTypes.PRISMATIC:
                    jacobian_matrix[0:3, j_index:j_index + 1] = np.matmul(rotation_matrix, joint_axis)
                j_index += 1
 
        return jacobian_matrix
    
    def get_forward_as_dict(self, output_type='numpy'):
        """
        Computes forward kinematics for a robot given a list of joint positions.

        Args:
            output_type (str): 'numpy' or 'torch' to specify the desired output format.
            
        Returns:
            a dictionary containing the forward kinematics for each link in the chain 
            as either numpy arrays or torch tensors.
        """
        if output_type not in ['numpy', 'torch']:
            raise ValueError("output_type deve essere 'numpy' o 'torch'")
        
        for i, link_names in enumerate(self.forward_dict.keys()):
            self.forward_dict[link_names] = self.forward_tensor[i]
        # if output_type == 'torch':
        #     # Conversione a torch tensor con dtype e device
        #     self.forward_dict = {
        #         self.robot_chain[i].name: torch.tensor(self.forward_tensor[i], device=self.device)
        #         for i in range(len(self.robot_chain))
        #     }
        # else:
        #     self.forward_dict = {self.robot_chain[i].name: self.forward_tensor[i] for i in range(len(self.robot_chain))}
        
        return self.forward_dict


    def jacobian_cog(self, joint_name: str):
        # Creazione di una mappa dal nome dei giunti agli indici
        joint_index_map = {link.name: i for i, link in enumerate(self.robot_chain)}
    
        # Controllo dell'esistenza del nome del giunto
        if joint_name not in joint_index_map:
            raise ValueError(f"Joint '{joint_name}' not found in the robot chain.")
    
        # Ottieni l'indice del giunto utilizzando il nome
        joint_i = joint_index_map[joint_name]
        
        # Inizializza la matrice jacobiana
        jacobian_matrix = np.zeros((3, self.num_of_joints), dtype=self.dtype)
        j_index = 0
        cog = self.robot_chain[joint_i].link.com
        cog_w = (self.forward_tensor[joint_i] @ np.array([cog[0], cog[1], cog[2], 1]))[:3] #attenzione potrebbe essere che si cava il +1
        # Itera attraverso i giunti fino al giunto di interesse
        for i in range(joint_i):  # +1 per includere il giunto stesso
            if self.robot_chain[i].joint.joint_type != JointTypes.FIXED:
                transformation_matrix = self.forward_tensor[i+1]
                # Matrice di rotazione del giunto
                rotation_matrix = transformation_matrix[0:3, 0:3]
                joint_axis_global = rotation_matrix @ self.robot_chain[i].joint.axis.flatten()
                
                
                if self.robot_chain[i].joint.joint_type == JointTypes.REVOLUTE:
                    # Velocità lineare indotta dal giunto rotazionale rispetto al CoG
                    jacobian_matrix[0:3, j_index:j_index + 1] = np.cross(joint_axis_global, (cog_w - transformation_matrix[0:3, 3:4].flatten())).reshape(3, 1)
                elif self.robot_chain[i].joint.joint_type == JointTypes.PRISMATIC:
                    # Contributo di un giunto prismatico al centro di massa
                    jacobian_matrix[0:3, j_index:j_index + 1] = joint_axis_global
                    
                j_index += 1
    
        return jacobian_matrix
    #da trasferire su un file per le dinamiche
    def gravity_compensation(self, q):
        gravity_force = np.zeros(3)
        gravity_torque = np.zeros(self.num_of_joints)
        for component in self.robot_chain: #RICORDATI DI SOMMARE PURE LE MASSE DEI JOINT FIXED QUANDO COLLASSI LA ROBOT CHAIN 
            gravity_force = GRAVITY_VECTOR * component.link.mass
            
            jacobian = self.jacobian_cog(component.link.name)
            # print("Name", component.link.name , "Jacobiano", jacobian)
           
            gravity_torque += np.dot(jacobian.T, gravity_force.reshape(3,1)).flatten()
        
        return gravity_torque