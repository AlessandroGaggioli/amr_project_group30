import numpy as np
from joint import Joint
from link import Link


class Component(object):
    def __init__(self, name=None, link=None, joint=None, device='cpu', dtype=np.float32):
        self.device = device
        self.dtype = dtype

        self.name = 'None' if name is None else name
        self.link = link if link is not None else Link()
        self.joint = joint if joint is not None else Joint()
        self.children = []
        self._identity_matrix_ = np.eye(4, dtype=self.dtype)

    def to(self, device='cpu'):


        self.device = device
        print("\033[92m" + 'Set robot component "{}" to {}'.format(self.name, self.device) + "\033[0m")

        return self

    def add_child(self, child):
        self.children.append(child)

    def get_visual_mesh_path(self):
        return self.link.get_visual_mesh_path()

    def get_visual_mesh_offset(self):
        return self.link.get_visual_mesh_offset()
    
    def get_collision_mesh_path(self):
        return self.link.get_collision_mesh_path()
    
    def get_collision_mesh_offset(self):
        return self.link.get_collision_mesh_offset()

    def get_number_of_children(self):
        return len(self.children)

    def is_end(self):
       return (self.children[-1].name == 'None')
    
    def get_mass(self):
        return self.link.mass
    
    def get_cog(self):
        return self.link.com

    def __repr__(self):
        # return "Component(name='{0}'\n link={1}\n joint={2}\n children={3})".format(self.name, self.link, self.joint, self.children)
        children_name_list = []
        for child in self.children:
            children_name_list.append(child.name)
        return "Name:{0}\n, children:{1}".format(self.name, children_name_list)