import numpy as np

class JointAngle_HomTf:
    
    def __init__(self, axis, dtype=np.float32, device='cpu'):

        self.dtype = dtype
        self.device = device
        self.h_matrix = np.zeros((4, 4), dtype=dtype)
        self.zero_tensor = np.array(0, dtype=dtype)
        self.one_tensor = np.array(1, dtype=dtype)

        self.cos_phi = np.array(0, dtype=dtype)
        self.sin_phi = np.array(0, dtype=dtype)
        self.cos_theta = np.array(0, dtype=dtype)
        self.sin_theta = np.array(0, dtype=dtype)
        self.cos_psi = np.array(0, dtype=dtype)
        self.sin_psi = np.array(0, dtype=dtype)
        
        self.axis = axis
        self.kx, self.ky, self.kz = self.axis
        
        self.compute(self.zero_tensor, self.zero_tensor, self.zero_tensor, self.zero_tensor, self.zero_tensor, self.zero_tensor)
    
    def to(self, dtype, device):
        # self.h_matrix = self.h_matrix.to(dtype=dtype, device=device)
        # self.zero_tensor = self.zero_tensor.to(dtype=dtype, device=device)
        # self.one_tensor = self.one_tensor.to(dtype=dtype, device=device)
        # self.cos_phi = self.cos_phi.to(dtype=dtype, device=device)
        # self.sin_phi = self.sin_phi.to(dtype=dtype, device=device)
        # self.cos_theta = self.cos_theta.to(dtype=dtype, device=device)
        # self.sin_theta = self.sin_theta.to(dtype=dtype, device=device)
        # self.cos_psi = self.cos_psi.to(dtype=dtype, device=device)
        # self.sin_psi = self.sin_psi.to(dtype=dtype, device=device)
        # self.axis = self.axis.to(dtype=dtype, device=device)
        # self.kx, self.ky, self.kz = torch.unbind(self.axis, -1)
        self.h_matrix = self.h_matrix.astype(dtype)
        self.zero_tensor = self.zero_tensor.astype(dtype)
        self.one_tensor = self.one_tensor.astype(dtype)
        self.cos_phi = self.cos_phi.astype(dtype)
        self.sin_phi = self.sin_phi.astype(dtype)
        self.cos_theta = self.cos_theta.astype(dtype)
        self.sin_theta = self.sin_theta.astype(dtype)
        self.cos_psi = self.cos_psi.astype(dtype)
        self.sin_psi = self.sin_psi.astype(dtype)
        self.axis = self.axis.astype(dtype)
        self.kx, self.ky, self.kz = self.axis
        return self
    
    def compute_revolute(self, joint_angle):
        # joint_angle = joint_angle.astype(np.float64)
        c = np.cos(joint_angle).astype(self.dtype)  # NOTE: cos is not that precise for float32, you may want to use float64
        one_minus_c = 1 - c
        # joint_angle = joint_angle.astype(np.float64)
        s = np.sin(joint_angle).astype(self.dtype)
        
        # start = time.time()
        self.h_matrix[0:3, 0:3] = np.array([[c + self.kx * self.kx * one_minus_c, self.kx * self.ky * one_minus_c - self.kz * s, self.kx * self.kz * one_minus_c + self.ky * s],
                                                [self.ky * self.kx * one_minus_c + self.kz * s, c + self.ky * self.ky * one_minus_c, self.ky * self.kz * one_minus_c - self.kx * s],
                                                [self.kz * self.kx * one_minus_c - self.ky * s, self.kz * self.ky * one_minus_c + self.kx * s, c + self.kz * self.kz * one_minus_c]])
        # end = time.time()
        # print('Time for revolute', end-start)
        # print('HHHHH matrix', self.h_matrix)

    def compute_prismatic(self, joint_angle):
        self.h_matrix[:3, -1] = self.axis * joint_angle
        #return self.h_matrix

    def compute_continuous(self, joint_angle):
        # joint_angle = joint_angle.astype(np.float64)
        c = np.cos(joint_angle).astype(self.dtype)
        s = np.sin(joint_angle).astype(self.dtype)
        self.h_matrix[0:3, 0:3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    
    def compute(self, x, y, z, psi, theta, phi):
        # start = time.time()
        phi = phi.astype(np.float64)
        theta = theta.astype(np.float64)
        psi = psi.astype(np.float64)
        self.cos_phi = np.cos(phi).astype(self.dtype)
        self.sin_phi = np.sin(phi).astype(self.dtype)
        self.cos_theta = np.cos(theta).astype(self.dtype)
        self.sin_theta = np.sin(theta).astype(self.dtype)
        self.cos_psi = np.cos(psi).astype(self.dtype)
        self.sin_psi = np.sin(psi).astype(self.dtype)
        
        self.h_matrix[:] = np.array([
                    [self.cos_phi*self.cos_theta, 
                    self.cos_phi*self.sin_theta*self.sin_psi - self.sin_phi*self.cos_psi, 
                    self.cos_phi*self.sin_theta*self.cos_psi + self.sin_phi*self.sin_psi, 
                    x],

                    [self.sin_phi*self.cos_theta, 
                    self.sin_phi*self.sin_theta*self.sin_psi + self.cos_phi*self.cos_psi,
                    self.sin_phi*self.sin_theta*self.cos_psi - self.cos_phi*self.sin_psi, 
                    y],

                    [-self.sin_theta,
                    self.cos_theta*self.sin_psi,
                    self.cos_theta*self.cos_psi,
                    z],

                    [self.zero_tensor, 
                        self.zero_tensor,
                        self.zero_tensor,
                        self.one_tensor]
                ])
        # end = time.time()
        # print('Time for compute hom tf', end-start)
        # return self.h_matrix
    
    def __matmul__(self, other):
        if isinstance(other, JointAngle_HomTf):
            return self.h_matrix @ other.h_matrix
        elif isinstance(other, torch.Tensor):
            return self.h_matrix @ other
        else:
            raise ValueError("The type of the other object is not valid")


def Homogeneus_Matrix_Euler(x, y, z, psi, theta, phi): #tested ok
    # device = x.device
    return np.array([
                    [np.cos(phi)*np.cos(theta), 
                    np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi), 
                    np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi), 
                    x],

                    [np.sin(phi)*np.cos(theta), 
                    np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi),
                    np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi), 
                    y],

                    [-np.sin(theta),
                    np.cos(theta)*np.sin(psi),
                    np.cos(theta)*np.cos(psi),
                    z],

                    [np.array(0),
                     np.array(0),
                     np.array(0),
                     np.array(1)] 
                ])

    

def Homogeneus_Matrix_Quaternion(x, y, z, qw, qx, qy, qz):
    return np.array([
                        [1 - 2*qy**2 - 2*qz**2, 
                        2*qx*qy - 2*qz*qw, 
                        2*qx*qz + 2*qy*qw, 
                        x],

                        [2*qx*qy + 2*qz*qw, 
                        1 - 2*qx**2 - 2*qz**2,
                        2*qy*qz - 2*qx*qw, 
                        y],

                        [2*qx*qz - 2*qy*qw,
                        2*qy*qz + 2*qx*qw,
                        1 - 2*qx**2 - 2*qy**2,
                        z],

                        [0,
                        0,
                        0,
                        1] 
                    ])

def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])
    
    

def homogeneous_to_pose_euler(matrix):
    translation = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]
    euler_angles = rotation_matrix_to_euler_angles(rotation_matrix)
    return translation, euler_angles

# def homogeneous_to_pose_quat(matrix):
#     translation = matrix[:3, 3]
#     rotation_matrix = matrix[:3, :3]
#     quaternion = rotation_matrix_to_quaternion(rotation_matrix)
#     return translation, quaternion

def homogeneousMatrix_to_vector(T):
    vec = np.zeros(6)

    # Coordinate traslazionali
    vec[0] = T[0, 3]
    vec[1] = T[1, 3]
    vec[2] = T[2, 3]

    # Angoli di rotazione (calcolati tramite atan2)
    vec[3] = np.arctan2(T[2, 1], T[2, 2])
    vec[4] = np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2))
    vec[5] = np.arctan2(T[1, 0], T[0, 0])

    return vec