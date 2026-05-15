
import PyKDL as kdl
import numpy as np
from robot_kins.chain_utils.urdf_parser_py import urdf
from robot_kins.chain_utils.urdf_parser_py.urdf_tree import treeFromFile


def get_link_masses(urdf_file):
        
        # Carica il modello URDF
        robot = urdf.URDF.from_xml_file(urdf_file)

        # Verifica che il robot sia stato caricato correttamente
        

        # Stampa le masse di tutti i link
        for link in robot.links:
            if link.inertial:  # Verifica se il link ha una descrizione di inerzia
                mass = link.inertial.mass
                com = link.inertial.origin.xyz
                print(f"Link: {link.name}, Massa: {mass:.3f} kg")
                print(f"Link: {link.name}, CoG: {com}")
            else:
                print(f"Link: {link.name}, Massa: Non definita")

        return com

def compute_jacobian(urdf_file, joint_positions, start_link='world', end_link='panda_link7'):
    # Carica il modello URDF
    ok, tree = treeFromFile(urdf_file)
    kdl_chain = tree.getChain(start_link, end_link)

    # Crea una struttura per le posizioni dei giunti
    joint_positions_kdl = kdl.JntArray(kdl_chain.getNrOfJoints())
    for i, pos in enumerate(joint_positions):
        joint_positions_kdl[i] = pos

    # Crea una struttura per immagazzinare il Jacobiano
    jacobian = kdl.Jacobian(kdl_chain.getNrOfJoints())

    # Crea un risolutore per il Jacobiano
    jac_solver = kdl.ChainJntToJacSolver(kdl_chain)

    # Calcola il Jacobiano
    jac_solver.JntToJac(joint_positions_kdl, jacobian)
    
    # Converte il risultato in un array numpy
    jacobian_np = np.zeros((jacobian.rows(), jacobian.columns()))
    for i in range(jacobian.rows()):
        for j in range(jacobian.columns()):
            jacobian_np[i, j] = jacobian[i, j]
    
    return jacobian_np

def compute_gravity_compensation(urdf_file, joint_positions, start_link='world', end_link='panda_link7'):

    # Carica il modello URDF
    ok, tree = treeFromFile(urdf_file)
    kdl_chain = tree.getChain(start_link, end_link)

    # Imposta il vettore di gravità
    gravity_vector = kdl.Vector(0, 0, 9.81)  # gravità verso il basso

    # Inizializza i parametri dinamici per calcolare la gravità
    dyn_params = kdl.ChainDynParam(kdl_chain, gravity_vector)
    
    
    # Crea una struttura per le posizioni dei giunti
    joint_positions_kdl = kdl.JntArray(kdl_chain.getNrOfJoints())
    for i, pos in enumerate(joint_positions):
        joint_positions_kdl[i] = pos

    # Crea una struttura per immagazzinare il risultato
    gravity_torque = kdl.JntArray(kdl_chain.getNrOfJoints())

    # Calcola il vettore di torques di compensazione di gravità
    dyn_params.JntToGravity(joint_positions_kdl, gravity_torque)
    
    # Converte il risultato in un array numpy
    gravity_torque_np = np.array([gravity_torque[i] for i in range(gravity_torque.rows())])
    
    return gravity_torque_np