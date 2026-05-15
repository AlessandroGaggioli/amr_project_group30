import numpy as np

'''
    relies on x y z w quaternion representation
'''

class Quaternion:
    def __init__(self, device='cpu', dtype=np.float32):
        self.device = device
        self.dtype  = dtype
        self.one    = np.array(1.0, dtype=dtype)
        self.four   = np.array(4.0, dtype=dtype)
        self.zero   = np.array(0.0, dtype=dtype)
        self.q      = np.array([1, 0, 0, 0], dtype=dtype)
    
    def set(self, x, y, z, w):
        self.q = np.array([w, x, y, z], dtype=self.dtype)
        return self

    def to(self, device, dtype):
        self.device = device
        self.dtype = dtype
        self.q = self.q.astype(dtype)
        return self
    
    @property
    def w(self):
        return self.q[0]

    @property
    def x(self):
        return self.q[1]

    @property
    def y(self):
        return self.q[2]

    @property
    def z(self):
        return self.q[3]

    def from_rotation_matrix(self, rotMatrix):

        r11, r12, r13 = rotMatrix[0, 0], rotMatrix[0, 1], rotMatrix[0, 2]
        r21, r22, r23 = rotMatrix[1, 0], rotMatrix[1, 1], rotMatrix[1, 2]
        r31, r32, r33 = rotMatrix[2, 0], rotMatrix[2, 1], rotMatrix[2, 2]

        # Calcola i valori iniziali dei quaternioni
        self.q[0] = np.sqrt(np.clip((r11 + r22 + r33 + self.one) / self.four, a_min=self.zero, a_max=None)) #w
        self.q[1] = np.sqrt(np.clip((r11 - r22 - r33 + self.one) / self.four, a_min=self.zero, a_max=None)) #x
        self.q[2] = np.sqrt(np.clip((-r11 + r22 - r33 + self.one) / self.four, a_min=self.zero, a_max=None)) #y
        self.q[3] = np.sqrt(np.clip((-r11 - r22 + r33 + self.one) / self.four, a_min=self.zero, a_max=None)) #z


        # Determina il quaternione dominante e aggiorna i segni
        if self.q[0] >= self.q[1] and self.q[0] >= self.q[2] and self.q[0] >= self.q[3]:
            self.q[0], self.q[1], self.q[2], self.q[3] = self.q[0], self.q[1] * np.sign(r32 - r23), self.q[2] * np.sign(r13 - r31), self.q[3] * np.sign(r21 - r12)
        elif self.q[1] >= self.q[0] and self.q[1] >= self.q[2] and self.q[1] >= self.q[3]:
            self.q[0], self.q[1], self.q[2], self.q[3] = self.q[0] * np.sign(r32 - r23), self.q[1], self.q[2] * np.sign(r21 + r12), self.q[3] * np.sign(r13 + r31)
        elif self.q[2] >= self.q[0] and self.q[2] >= self.q[1] and self.q[2] >= self.q[3]:
            self.q[0], self.q[1], self.q[2], self.q[3] = self.q[0] * np.sign(r13 - r31), self.q[1] * np.sign(r21 + r12), self.q[2], self.q[3] * np.sign(r32 + r23)
        elif self.q[3] >= self.q[0] and self.q[3] >= self.q[1] and self.q[3] >= self.q[2]:
            self.q[0], self.q[1], self.q[2], self.q[3] = self.q[0] * np.sign(r21 - r12), self.q[1] * np.sign(r31 + r13), self.q[2] * np.sign(r32 + r23), self.q[3]
        else:
            raise ValueError("Coding error")


        # Crea il tensore del quaternione e normalizzalo
        self.q = np.stack([self.q[0], self.q[1], self.q[2], self.q[3]])
        
        
        self.q /= np.linalg.norm(self.q)

        return self.q
    
    @staticmethod
    def quaternion_to_rotation_matrix(q):
        print(q)
        w, x, y, z = q
        R = np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
        ]).astype(Quaternion.config["dtype"])
        return R

    
    def slerp(self, target_quat, t):
 
        q1 = self.q / np.linalg.norm(self.q, ord=2, axis=-1, keepdims=True)
        q2 = target_quat / np.linalg.norm(target_quat, ord=2, axis=-1, keepdims=True)

        dot = np.dot(q1, q2)
        if dot < self.zero:
            q1 = -q1
            dot = -dot

        vdiff = q2 - q1
        DOT_THRESHOLD = 0.9995
        if dot > DOT_THRESHOLD:
            result = q1 + t * vdiff
            result = result / np.linalg.norm(result, ord=2, axis=-1, keepdims=True)
            return result
        
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return (s0 * q1) + (s1 * q2)


    def rotation_velocity(self, target_quat, cycleHz):
        new_target_quat = self.slerp(target_quat, 1/cycleHz)
        dot_product = np.dot(self.q, new_target_quat)

        if dot_product < 0:
            new_target_quat = -new_target_quat

        # Matrice E
        E = np.array([
            [-self.q[1],  self.q[0], -self.q[3],  self.q[2]],
            [-self.q[2],  self.q[3],  self.q[0], -self.q[1]],
            [-self.q[3], -self.q[2],  self.q[1],  self.q[0]]
        ]).astype(self.dtype)

        # Calcolo dell'errore del quaternionr

        return 2 * cycleHz * np.matmul(E, (new_target_quat - self.q))
        
        
    def from_rpy(self, roll, pitch, yaw):
        '''
            q0 = w , q1 = x, q2 = y, q3 = z
        '''
        self.q[1] = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) # x
        self.q[2] = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) # y
        self.q[3] = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) # z
        self.q[0] = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) # w
        return self.q.copy()
    
    def to_euler(self):
        t0 = +2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3])
        t1 = +1.0 - 2.0 * (self.q[1]**2 + self.q[2]**2)
        roll_x = torch.atan2(t0, t1)

        t2 = +2.0 * (self.q[0] * self.q[2] - self.q[3] * self.q[1])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (self.q[0] * self.q[3] + self.q[1] * self.q[2])
        t4 = +1.0 - 2.0 * (self.q[2]**2 + self.q[3]**2)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    # def normalize(self):
    #     self.q = self.q / torch.norm(self.q)
    #     return self

    # def inverse(self):
    #     w, x, y, z = self.q
    #     norm_sq = torch.sum(self.q ** 2)
    #     self.q = torch.tensor([w, -x, -y, -z]) / norm_sq
    #     return self
    # def conjugate(self):
    #     return Quaternion(self.w, -self.x, -self.y, -self.z)

    # def angular_distance(self, other):
    #     dot_product = self.dot(other)
    #     return 2 * torch.acos(torch.clamp(dot_product, -1.0, 1.0))
    
    # def dot(self, other):
    #     return torch.dot(self.q, other.q)

    # def __mul__(self, other):
    #     w0, x0, y0, z0 = self.q
    #     w1, x1, y1, z1 = other.q
    #     return Quaternion(
    #         w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
    #         w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
    #         w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
    #         w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    #     )

    # @staticmethod
    # def negation(v):
    #     return Quaternion(-v.w, -v.x, -v.y, -v.z)

    # @staticmethod
    # def scalar_product(v, t):
    #     return Quaternion(t * v.w, t * v.x, t * v.y, t * v.z)

    # @staticmethod
    # def minus(v1, v0):
    #     return Quaternion(v1.w - v0.w, v1.x - v0.x, v1.y - v0.y, v1.z - v0.z)

    # @staticmethod
    # def plus(v1, v0):
    #     return Quaternion(v1.w + v0.w, v1.x + v0.x, v1.y + v0.y, v1.z + v0.z)

    
    def __repr__(self):
        return f"Quaternion(w={self.w}, x={self.x}, y={self.y}, z={self.z})"
    


