import numpy as np

def Rx(gamma):
    return np.array([[1, 0 ,          0 ,            0],
                         [0, np.cos(gamma), -np.sin(gamma),    0],
                         [0, np.sin(gamma), np.cos(gamma),     0],
                         [0, 0,          0,              1] 
                        ])

def Ry(beta):
    return np.array([[np.cos(beta),  0 , np.sin(beta) ,   0],
                         [0,           1,  0,             0],
                         [-np.sin(beta), 0,  np.cos(beta),    0],
                         [0,           0,  0,             1] 
                        ])

def Rz(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha), 0,   0],
                         [np.sin(alpha), np.cos(alpha),  0,   0],
                         [0,          0,           1,   0],
                         [0,          0,           0,   1] 
                        ])

def RotationMatrix_ZYZ_Convention(phi = None, theta = None, psi = None):
    try:
        return Rz(psi) @ Ry(theta) @ Rz(phi)
    except: 
        print("This function has three arguments Required")
    return

def RotationMatrix_ZYX_Convention(phi = None, theta = None, psi = None):
    try:
        return Rz(psi) @ Ry(theta) @ Rx(phi)
    except: 
        print("This function has three arguments Required")
    return

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x**2 + y**2)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y**2 + z**2)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def euler_to_quaternion(roll_x, pitch_y, yaw_z):
    qx = np.sin(roll_x/2) * np.cos(pitch_y/2) * np.cos(yaw_z/2) - np.cos(roll_x/2) * np.sin(pitch_y/2) * np.sin(yaw_z/2)
    qy = np.cos(roll_x/2) * np.sin(pitch_y/2) * np.cos(yaw_z/2) + np.sin(roll_x/2) * np.cos(pitch_y/2) * np.sin(yaw_z/2)
    qz = np.cos(roll_x/2) * np.cos(pitch_y/2) * np.sin(yaw_z/2) - np.sin(roll_x/2) * np.sin(pitch_y/2) * np.cos(yaw_z/2)
    qw = np.cos(roll_x/2) * np.cos(pitch_y/2) * np.cos(yaw_z/2) + np.sin(roll_x/2) * np.sin(pitch_y/2) * np.sin(yaw_z/2)
    q = np.array([qx, qy, qz, qw])
    return q
    

def rotation_velocity(cycleHz, q1, q2):
    # Estrazione dei quaternioni
    quat = np.array([q1[3], q1[0], q1[1], q1[2]])
    obj  = np.array([q2[3], q2[0], q2[1], q2[2]])

    new_target_quat = slerp(quat, obj, 1/cycleHz)
    dot_product = np.dot(quat, new_target_quat)

    if dot_product < 0:
        new_target_quat = -new_target_quat

    # Matrice E
    E = torch.tensor([
        [-quat[1],  quat[0], -quat[3],  quat[2]],
        [-quat[2],  quat[3],  quat[0], -quat[1]],
        [-quat[3], -quat[2],  quat[1],  quat[0]]
    ], dtype=np.float32)

    # Calcolo dell'errore del quaternionr

    w = 2 * cycleHz * np.matmul(E, (new_target_quat - quat))
    
    return w

def slerp(q1:torch.Tensor, q2:torch.Tensor, t):
    q1 = q1 / np.linalg.norm(q1, ord=2, axis=-1, keepdims=True)
    q2 = q2 / np.linalg.norm(q2, ord=2, axis=-1, keepdims=True)

    dot = np.dot(q1, q2)
    if dot < 0.0:
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

def rotation_matrix_to_quaternion(rotMatrix):
    # Estrai gli elementi della matrice di rotazione
    r11, r12, r13 = rotMatrix[0, 0], rotMatrix[0, 1], rotMatrix[0, 2]
    r21, r22, r23 = rotMatrix[1, 0], rotMatrix[1, 1], rotMatrix[1, 2]
    r31, r32, r33 = rotMatrix[2, 0], rotMatrix[2, 1], rotMatrix[2, 2]

    # Calcola i valori iniziali dei quaternioni
    q0 = np.sqrt(np.clip((r11 + r22 + r33 + 1.0) / 4.0, 0.0, None))
    q1 = np.sqrt(np.clip((r11 - r22 - r33 + 1.0) / 4.0, 0.0, None))
    q2 = np.sqrt(np.clip((-r11 + r22 - r33 + 1.0) / 4.0, 0.0, None))
    q3 = np.sqrt(np.clip((-r11 - r22 + r33 + 1.0) / 4.0, 0.0, None))

    # Determina il quaternione dominante e aggiorna i segni
    if q0 >= q1 and q0 >= q2 and q0 >= q3:
        q0, q1, q2, q3 = q0, q1 * np.sign(r32 - r23), q2 * np.sign(r13 - r31), q3 * np.sign(r21 - r12)
    elif q1 >= q0 and q1 >= q2 and q1 >= q3:
        q0, q1, q2, q3 = q0 * np.sign(r32 - r23), q1, q2 * np.sign(r21 + r12), q3 * np.sign(r13 + r31)
    elif q2 >= q0 and q2 >= q1 and q2 >= q3:
        q0, q1, q2, q3 = q0 * np.sign(r13 - r31), q1 * np.sign(r21 + r12), q2, q3 * np.sign(r32 + r23)
    elif q3 >= q0 and q3 >= q1 and q3 >= q2:
        q0, q1, q2, q3 = q0 * np.sign(r21 - r12), q1 * np.sign(r31 + r13), q2 * np.sign(r32 + r23), q3
    else:
        raise ValueError("Coding error")

    # Crea il vettore del quaternione e normalizzalo
    q = np.array([q1, q2, q3, q0])
    q /= np.linalg.norm(q)

    return q


