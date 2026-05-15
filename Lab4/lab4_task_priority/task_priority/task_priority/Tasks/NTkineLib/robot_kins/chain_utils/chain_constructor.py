import os, sys
sys.path.append(os.path.dirname(__file__))
import numpy as np
from urdf_parser_py.urdf import URDF
from urdf_parser_py.urdf import Box, Cylinder, Sphere, Mesh
from robot_components.link import Link, Visual, Collision 
from robot_components.joint import Joint
from robot_components.frame import Component




class ChainConstructor:
    def __init__(self, urdf_file, device='cpu', dtype=np.float32):
        """
        Inizializza il robot caricando la struttura del file URDF e imposta il dispositivo e tipo dati.
        """
        self.device = device
        self.dtype = dtype
        
        # Carica il file URDF
        self.robot = URDF.from_xml_file(urdf_file)


    def __joint_contructor(self, joint):
        try:
            origin = joint.origin.xyz + joint.origin.rpy
        except:
            origin = np.zeros(6)

        joint_comp = Joint(name=joint.name, 
                           frame=origin, 
                           joint_type=joint.type, 
                           axis=joint.axis, 
                           dtype=self.dtype, 
                           device=self.device
                           )
        
        joint_comp.set_dynamic_properties(joint.limit, joint.dynamics)
        joint_comp.set_child_parent(joint.child, joint.parent)
        return joint_comp


    def __link_constructor(self, link):

        try:
            origin = link.origin.xyz + link.origin.rpy
        except:
            origin = np.zeros(6)


        try:    
            mass = link.inertial.mass
        except:
            mass = 0.0
        try:
            # Unpack inertia
            inertia_matrix = np.array([[float(link.inertial.inertia.ixx), float(link.inertial.inertia.ixy), float(link.inertial.inertia.ixz)],
                                    [float(link.inertial.inertia.ixy), float(link.inertial.inertia.iyy), float(link.inertial.inertia.iyz)],
                                    [float(link.inertial.inertia.ixz), float(link.inertial.inertia.iyz), float(link.inertial.inertia.izz)]])
        except:
            inertia_matrix = np.zeros((3,3))
        try:
            cog = link.inertial.origin.xyz + link.inertial.origin.rpy
        except:
            cog = np.zeros(6)


        try:
            visual_origin = np.array(link.visual.origin.xyz + link.visual.origin.rpy)
        except:
            visual_origin = np.zeros(6)
        try:
            collision_origin = np.array(link.collision.origin.xyz + link.collision.origin.rpy)
        except:
            collision_origin = np.zeros(6)

        try:
            geometry = link.visual.geometry
            if isinstance(geometry, Box):
                geometry_type = "box"
                geom_param = [geometry.size[0], geometry.size[1], geometry.size[2]]
            elif isinstance(geometry, Cylinder):
                geometry_type = "cylinder"
            elif isinstance(geometry, Sphere):
                geometry_type = "sphere"
            elif isinstance(geometry, Mesh):
                geometry_type = "mesh" 
                geom_param = None       
        except:
            geometry_type = None
            geom_param = None
        
        try:
            mesh_visual_path = link.visual.geometry.filename
        except:
            mesh_visual_path = None

        try:
            mesh_collision_path = link.collision.geometry.filename
        except:
            mesh_collision_path = None

        # print("geom_param: ", geom_param)
        
        try:
            # print(link.visual.origin)
            
            # print(visual_origin)
            #print(link.visual.geometry)

            visual = Visual(offset=visual_origin, geom_type=geometry_type, geom_param=geom_param, mesh_path=mesh_visual_path)
            
            collision = Collision(mesh_path=mesh_collision_path, offset=collision_origin)
        except:
            visual = Visual()
            collision = Collision()
            mesh = None
            
        link_comp = Link(name=link.name, offset=origin, visuals=visual, collision=collision, dtype=self.dtype, device=self.device)
        link_comp.set_mass_properties(mass, inertia_matrix, cog)

        return link_comp

    def __frame_constructor(self, robot, link, joint):
        # print(f"Link: {link}, Joint: {joint}")
        link_comp = Link()
        joint_comp = Joint()
        for component in robot.links:
            if component.name == link:
                link_comp = self.__link_constructor(component)
                break
        for component in robot.joints:
            if component.name == joint:
                joint_comp = self.__joint_contructor(component)
                break

        return Component(name=link_comp.name, link=link_comp, joint=joint_comp)
    
    def get_chain(self, start_link, end_link):
        chain = self.robot.get_chain(start_link, end_link)
        # print(f"Chain: {chain}")
        
        robot_chain = []
        for chain_link, joint_link in zip(chain[::2], chain[1::2]):
            # print(f"Chain Link: {chain_link}, Joint Link: {joint_link}")
            component = self.__frame_constructor(self.robot, chain_link, joint_link)


            pair_childs = self.robot.child_map[chain_link]
            for child_joint, child_link in pair_childs:
                # print(f"Child Link: {child_link}, Child Joint: {child_joint}")
                child = self.__frame_constructor(self.robot, child_link, child_joint)
                component.add_child(child)
            
            robot_chain.append(component)
        
        robot_chain.append( self.__frame_constructor(self.robot, chain[-1], None))
        
        return robot_chain


if __name__ == "__main__":
    robot = ChainConstructor(urdf_file='test/panda.urdf')

    chain = robot.get_chain('panda_link0', 'panda_link7')

    for chain_link in chain:
        print(chain_link)
        print("\n")
