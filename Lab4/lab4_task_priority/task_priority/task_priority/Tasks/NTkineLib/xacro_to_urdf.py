import xacro, os
from xml.dom import minidom

def convert_xacro_to_urdf(input_file, output_path=None, xacro_args=None): 
    """
        Converte un file .xacro in un file .urdf e sostituisce gli argomenti specificati.

        Args:
            input_file (str): Percorso al file .xacro di input.
            output_path (str, opzionale): Percorso in cui salvare il file .urdf convertito.
            xacro_args (dict, opzionale): Dizionario di argomenti xacro da sostituire.

        Returns:
            str: Percorso al file .urdf convertito.
    """

    print(f'\033[92m' + f'Converting xacro file: {input_file} to urdf' + "\033[0m")
    print(f'\033[93m' + f'Output path: {output_path}' + "\033[0m")
    #print the arguments that will be replaced
    if xacro_args:
        print("Using the following xacro arguments:")
        for key, value in xacro_args.items():
            print(f"{key} = {value}")
    
    file_name = os.path.basename(input_file)
    file_extention = os.path.splitext(file_name)[-1]
    file_name = os.path.splitext(file_name)[0]
    file_path = os.path.dirname(input_file)
    

    if file_extention == '.xacro':
        # Processa il file .xacro con gli argomenti forniti
        robot_description_config = xacro.process_file(input_file, mappings=xacro_args)
        robot_desc = robot_description_config.toxml()
        xml_pretty_str = minidom.parseString(robot_desc).toprettyxml(indent="  ")
        # Determina il nome del file di output
        # if os.path.splitext(file_name)[-1] != '.urdf':
        #     input_file = os.path.join(file_path, file_name + ".urdf")
        # else:
        #     input_file = os.path.join(file_path, file_name)
        
        # if output_path is not None:
        #     input_file = os.path.join(output_path, file_name + ".urdf")

        with open(os.path.abspath(output_path), "w") as file_open:
            file_open.write(xml_pretty_str)

    return input_file


if __name__ == "__main__":
    xacro_file = os.path.join(os.path.dirname(__file__), "test", "panda.xacro")
    output_path = os.path.join(os.path.dirname(__file__), "test", "panda.urdf")
    xacro_args = {
        'robot_description': 'panda',
        'robot_name': 'panda',
        'robot_namespace': 'panda'
    }
    convert_xacro_to_urdf(xacro_file, output_path, xacro_args)