import numpy as np

class Task:
    def __init__(self, name, priority, num_of_joints, device='cpu', dtype=np.float32):
        self.name     = name
        # self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.device = device
        self.dtype    = dtype
        self.device   = device
        self.priority = priority
        self.dof         = num_of_joints
        self.joint_vel   = np.zeros(num_of_joints, dtype=self.dtype)
        self.A_ = np.eye(num_of_joints, num_of_joints, dtype=self.dtype)
        self.jacobian = np.zeros((6, num_of_joints), dtype=self.dtype)
        self.weighted_projector = np.zeros((num_of_joints, num_of_joints), dtype=self.dtype)
        self.identity = np.eye(num_of_joints, dtype=self.dtype)

    def to(self, device, dtype):
        # self.A = self.activation_matrix.to(device=device, dtype=dtype)
        # self.jacobian = self.jacobian.to(device=device, dtype=dtype)
        # self.weighted_projector = self.weighted_projector.to(device=device, dtype=dtype)
        # self.identity = self.identity.to(device=device, dtype=dtype)
        # self.ee_motion_task_constraint = self.ee_motion_task_constraint.to(device=device, dtype=dtype)
        # self.joint_motion_task_constraint = self.joint_motion_task_constraint.to(device=device, dtype=dtype)
        return self

    def show(self):
        pass
    
    def set_motion_param(self):
        pass  
    
    def get_num_of_dof(self):
        return self.dof
        
    def get_name(self):
        return self.name
    
    def set_activation_matrix(self, activation_matrix):
        raise NotImplementedError("This method should be overridden by subclasses probably")
    
    def update_jacobian(self, jacobian):
        self.jacobian = jacobian

    def update_theta_ts(self, theta_ts):
        self.theta_ts = theta_ts
    
    def compute_projector(self):
        raise NotImplementedError("This method should be overridden by subclasses")
    
    def compute_weighted_projector(self):
        pass 
    
    def execute(self):
        raise NotImplementedError("This method should be overridden by subclasses")
    
    def get_activation_matrix(self):
        return self.A
    
  