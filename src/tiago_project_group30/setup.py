from setuptools import find_packages, setup
from glob import glob
import os 

package_name = 'tiago_project_group30'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alessandrogaggioli',
    maintainer_email='alessandrogaggioli@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task1_manager = tiago_project_group30.task1_manager:main',
            'task2_manager = tiago_project_group30.task2_manager:main',
            'task3_manager = tiago_project_group30.task3_manager:main',
        ],
    },
)
