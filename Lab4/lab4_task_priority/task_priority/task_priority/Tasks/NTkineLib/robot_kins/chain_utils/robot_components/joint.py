import numpy as np
from robot_components.transforms.transforms import Homogeneus_Matrix_Euler, JointAngle_HomTf, homogeneousMatrix_to_vector

class JointTypes:
    # TYPES = ['fixed', 'revolute', 'prismatic']
    FIXED = 1
    REVOLUTE = 2
    PRISMATIC = 3
    CONTINUOUS = 4

    @staticmethod
    def get_corr_enum(joint_type):
        if joint_type == 'fixed':
            return JointTypes.FIXED
        elif joint_type == 'revolute':
            return JointTypes.REVOLUTE
        elif joint_type == 'prismatic':
            return JointTypes.PRISMATIC
        elif joint_type == 'continuous':
            return JointTypes.CONTINUOUS
        else:
            raise RuntimeError("joint specified as {} type not, but we only support {}".format(joint_type, ['fixed', 'revolute', 'prismatic']))

    @staticmethod
    def get_enum_string(enum_value):
        if enum_value == JointTypes.FIXED:
            return 'fixed'
        elif enum_value == JointTypes.REVOLUTE:
            return 'revolute'
        elif enum_value == JointTypes.PRISMATIC:
            return 'prismatic'
        elif enum_value == JointTypes.CONTINUOUS:
            return 'continuous'
        else:
            raise RuntimeError("Invalid joint type enum value: {}".format(enum_value))

class Joint(object):
        
    @classmethod
    def _initialize_frame_(cls, frame, dtype=np.float32):
        return Homogeneus_Matrix_Euler(frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]).astype(dtype)
    
    def __init__(self, name='None', frame=np.zeros(6), joint_type='fixed', axis=None, dtype=np.float32, device='cpu'):
        self.dtype = dtype
        # ADDED this fix
        self.device = device

        self.name = name

        if isinstance(frame, list):
            frame = np.array(frame, dtype=self.dtype)
        elif frame is None:
            frame = np.array([0.0]*6, dtype=self.dtype)

        self.tf = self._initialize_frame_(frame, dtype=self.dtype)
        
        self.joint_type = JointTypes.get_corr_enum(joint_type)
        
        if axis is None:
            self.axis = np.array([0.0, 0.0, 0.0], dtype=self.dtype)
        else:
            self.axis = np.array(axis, dtype=self.dtype)
        
        self.axis = np.round(self.axis).astype(self.dtype)
        
        
        self.lower_lim    = np.array(0.0, dtype=self.dtype)
        self.upper_lim    = np.array(0.0, dtype=self.dtype)
        self.damping      = np.array(0.0, dtype=self.dtype)
        self.friction     = np.array(0.0, dtype=self.dtype)
        self.effort_lim   = np.array(0.0, dtype=self.dtype)
        self.velocity_lim = np.array(0.0, dtype=self.dtype)
        self.zero         = np.array(0.0, dtype=self.dtype)
        self.joint_angle_contribution_tf = JointAngle_HomTf(self.axis, dtype=self.dtype, device=self.device)

        self.parent = None
        self.child = None
        self.tf_joint_increment = np.eye(4, dtype=self.dtype)
        
    def set_dynamic_properties(self, limits=None, dynamics=None):
        if limits is not None:
            #if limits are not None -> xml parser needs effort, velocity, lower and upper limits to be set otherwise it raises an error
            self.lower_lim = np.array(limits.lower, dtype=self.dtype)
            self.upper_lim = np.array(limits.upper, dtype=self.dtype)
            self.effort_lim = np.array(limits.effort, dtype=self.dtype)
            self.velocity_lim = np.array(limits.velocity, dtype=self.dtype)
        if dynamics is not None:
            self.damping = np.array(0 if dynamics.damping is None else dynamics.damping, dtype=self.dtype)
            self.friction = np.array(0 if dynamics.friction is None else dynamics.friction, dtype=self.dtype)
    
    def set_child_parent(self, child, parent):
        self.child = child
        self.parent = parent

    def to(self, *args, **kwargs):

        dtype = list(kwargs.values())[1]
        self.lower_lim = self.lower_lim.astype(dtype)
        self.upper_lim = self.upper_lim.astype(dtype)
        self.axis = self.axis.astype(dtype)
        self.tf = self.tf.astype(dtype)
        self.damping = self.damping.astype(dtype)
        self.friction = self.friction.astype(dtype)
        self.effort_lim = self.effort_lim.astype(dtype)
        self.velocity_lim = self.velocity_lim.astype(dtype)
        self.zero = self.zero.astype(dtype)
        self.joint_angle_contribution_tf = self.joint_angle_contribution_tf.to(*args, **kwargs)
        return self

    def clamp(self, joint_position):
        if self.joint_type == 'fixed':
            return joint_position
        elif joint_position < self.lower_lim or joint_position > self.upper_lim:
            RuntimeWarning("Joint exceed the limit, clamping joint position")
        
        return torch.clamp(joint_position, self.lower_lim, self.upper_lim)
        
    def compute_joint_increment(self, joint_position=None):
        if self.joint_type == JointTypes.FIXED:
            return self.tf
        elif self.joint_type == JointTypes.REVOLUTE:
            self.joint_angle_contribution_tf.compute_revolute(joint_position)
            return self.tf @ self.joint_angle_contribution_tf.h_matrix
        elif self.joint_type == JointTypes.PRISMATIC:
            self.joint_angle_contribution_tf.compute_prismatic(joint_position)
            return self.tf @ self.joint_angle_contribution_tf.h_matrix
        elif self.joint_type == JointTypes.CONTINUOUS:
            self.joint_angle_contribution_tf.compute_continuous(joint_position)
            return  self.tf @ self.joint_angle_contribution_tf.h_matrix 
    
    def get_type(self):
        return JointTypes.get_enum_string(self.joint_type)
                  
    def __repr__(self):
        return "Joint(name='{0}' \n transform={1}\n joint_type='{2}'\n axis={3}\n lower_lim={4}\n upper_lim={5}\n damping={6}\n friction={7}\n effort={8}\n velocity={9}\n parent={10}\n child={11})".format(
            self.name,
            self.tf,
            self.joint_type,
            self.axis,
            self.lower_lim,
            self.upper_lim,
            self.damping,
            self.friction,
            self.effort_lim,
            self.velocity_lim,
            self.parent,
            self.child)