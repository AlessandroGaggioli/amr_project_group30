from robot_kins.chain_utils.serial_kin import ChainKins
import numpy as np
from copy import deepcopy


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

def merge_chains(list_of_chains):
        common_link_name = find_common_link(list_of_chains)
        merged_chain = deepcopy(list_of_chains[0])
        for i in range(1, len(list_of_chains)):
            # Ottieni l'ultima trasformazione dell'ultima catena unita
            last_link_merged = merged_chain[-1]
            last_transform_merged = last_link_merged.joint.tf

            # Ottieni l'indice del link comune nella nuova catena
            current_chain = list_of_chains[i]
            idx_common = next(
                idx for idx, link in enumerate(current_chain) if link.name == common_link_name
            )

            # Calcola la trasformazione relativa del primo link dopo il link comune
            relative_transform = current_chain[idx_common + 1].joint.tf
            
            adjusted_transform = np.dot(last_transform_merged, relative_transform)

            # Aggiungi il link successivo con la trasformazione aggiornata
            new_link = deepcopy(current_chain[idx_common + 1])
            print(f"Aggiunto link: {new_link}")
            new_link.transform = adjusted_transform
            merged_chain.append(new_link)

            # Aggiungi i restanti link della catena corrente
            merged_chain.extend(deepcopy(current_chain[idx_common + 2 :]))
