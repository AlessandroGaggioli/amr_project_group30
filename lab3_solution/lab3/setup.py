from setuptools import find_packages, setup
from glob import glob
import os

package_name = "lab3"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "1_aruco_tf_sub = lab3.1_aruco_tf_subscriber:main",
            "2_aruco_grap_pose_broadcaster = lab3.2_aruco_grasp_pose_broadcaster:main",
            "3_move_arm = lab3.3_move_arm:main",
        ],
    },
)
