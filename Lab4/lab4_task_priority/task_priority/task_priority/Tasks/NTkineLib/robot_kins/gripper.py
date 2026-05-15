import os, sys
__WS_PATH__ = os.path.abspath(os.path.join(__file__, "../.."))
sys.path.append(__WS_PATH__) if __WS_PATH__ not in sys.path else None
from robot_kins.chain_utils.multiple_serial_kins import MultipleChainKins
import numpy as np

def find_common_link(list_of_chains):
    """
    Trova l'ultimo link comune tra tutte le catene.
    
    list_of_chains: list of Chain - Lista di catene.
        Ogni oggetto Chain ha un attributo `robot_chain` che è una lista di link.
    
    Ritorna:
        common_link_name: str - Nome dell'ultimo link comune.
    
    Solleva:
        ValueError - Se non ci sono link comuni tra tutte le catene.
    """
    # Estrai i set di nomi dei link per ciascuna catena
    chain_links = [{link.name for link in chain} for chain in list_of_chains]

    # Trova l'intersezione tra tutti i set di link
    common_links = set.intersection(*chain_links)
    if not common_links:
        raise ValueError("Nessun link comune trovato tra le catene.")

    # Trova l'ultimo link comune (in base all'ordine nella prima catena)
    for link in reversed(list_of_chains[0]):
        if link.name in common_links:
            return link.name

    raise ValueError("Errore imprevisto: nessun link comune trovato nella prima catena.")

class ParallelGripper(MultipleChainKins):
    def __init__(self, urdf_file, start_link, end_links,  device='cpu', dtype=np.float32):
        super().__init__(urdf_file, start_link, end_links, device, dtype)

        self.start_link = start_link
        self.num_of_joints = 0
        
    

    def compute(self, width, base_position):
        print(width)
        if isinstance(width, list):
            width = width[0]
        joint_list = [width/2, -width/2]

        self.fk(joint_list, base_link_position=base_position)
        # self.jacobian_matrix = np.zeros((6, self.num_of_joints), dtype=self.dtype)
        
        
            





if __name__ == "__main__":
    urdf = "test/tiago.urdf"
    start_link = "gripper_link"
    end_links = ["gripper_left_finger_link", "gripper_right_finger_link"]
    
    parallel_gripper = ParallelGripper(urdf, start_link, end_links[0], end_links[1])


    