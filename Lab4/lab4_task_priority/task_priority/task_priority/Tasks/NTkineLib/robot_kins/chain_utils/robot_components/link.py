import os, sys
__CURRENT_DIR__ = os.path.dirname(os.path.abspath(__file__))
__PARENT_DIR__ = os.path.dirname(__CURRENT_DIR__)
sys.path.append(__PARENT_DIR__) if __PARENT_DIR__ not in sys.path else None
sys.path.append(__CURRENT_DIR__) if __CURRENT_DIR__ not in sys.path else None
import numpy as np
from transforms.transforms import Homogeneus_Matrix_Euler

class Visual(object):
    TYPES = ['box', 'cylinder', 'sphere', 'capsule', 'mesh']

    def __init__(self, offset=None, geom_type=None, geom_param=None, mesh_path=None):
        if offset is None:
            self.offset = np.zeros(6)
        else:
            self.offset = offset

        self.geom_type = geom_type
        self.geom_param = geom_param
        self.mesh_path = mesh_path

    def get_mesh_path(self):
        return self.mesh_path
    
    def get_offset(self):
        return self.offset
    
    def get_mesh_geometry(self):
        return self.geom_type, self.geom_param

    def __repr__(self):
        return "Visual(offset={0}, geom_type='{1}', geom_param={2})".format(self.offset,
                                                                            self.geom_type,
                                                                            self.geom_param)
class Collision(object):
    def __init__(self, offset=None, geom_type=None, geom_param=None, mesh_path=None):
        if offset is None:
            self.offset = np.zeros(6)
        else:
            self.offset = offset

        self.geom_type = geom_type
        self.geom_param = geom_param
        self.mesh_path = mesh_path
    
    def get_mesh_path(self):
        return self.mesh_path
    
    def get_offset(self):
        return self.offset
    
    def get_mesh_geometry(self):
        return self.geom_type, self.geom_param

    def __repr__(self):
        return "Visual(offset={0}, geom_type='{1}', geom_param={2})".format(self.offset,
                                                                            self.geom_type,
                                                                            self.geom_param)

    def __repr__(self):
        return "Collision(name='{0}'\n offset={1})".format(self.name, self.offset)

class Link(object):
    def __init__(self, name='None', offset=np.zeros(6), visuals=(), collision=(), dtype=np.float32, device='cpu'):
        self.device = device
        self.dtype = dtype
        # if isinstance(offset, np.ndarray):
        #     offset = torch.from_numpy(offset).float()
        # elif isinstance(offset, list):
        #     offset = torch.tensor(offset, dtype=self.dtype)
        if isinstance(offset, list):
            offset = np.array(offset,  dtype=self.dtype)
        elif offset is None:
            offset = np.zeros(6, dtype=self.dtype)
        
        self.offset = Homogeneus_Matrix_Euler(offset[0], offset[1], offset[2], offset[3], offset[4], offset[5]).astype(self.dtype)
        self.mass = np.array(0.0, dtype=self.dtype)
        self.inertia = np.zeros((3, 3), dtype=self.dtype)
        self.com = np.zeros(6, dtype=self.dtype)

        self.name = name
        self.visuals = visuals
        self.collision = collision

    def set_mass_properties(self, mass, inertia, com):
        self.mass = np.array(mass, dtype=self.dtype)
        self.com = np.array(com, dtype=self.dtype)


    def to(self, *args, **kwargs):


        dtype = list(kwargs.values())[1]
        # self.offset = self.offset.to(*args, **kwargs)
        # self.mass = self.mass.to(*args, **kwargs)
        # self.inertia = self.inertia.to(*args, **kwargs)
        # self.com = self.com.to(*args, **kwargs)
        self.offset = self.offset.astype(dtype)
        self.mass = self.mass.astype(dtype)
        self.inertia = self.inertia.astype(dtype)
        self.com = self.com.astype(dtype)

        return self
    
    def get_visual_mesh_path(self):
        return self.visuals.get_mesh_path()
    
    def get_visual_mesh_offset(self):
        return self.visuals.get_offset()

    def get_collision_mesh_path(self):
        return self.collision.get_mesh_path()
    
    def get_collision_mesh_offset(self):
        return self.collision.get_offset()

    def __repr__(self):
        return "Link(name='{0}'\n offset={1}\n mass={2}\n inertia={3}\n com={4})".format(self.name,
                                                                                     self.offset,
                                                                                     self.mass,
                                                                                     self.inertia,
                                                                                     self.com)