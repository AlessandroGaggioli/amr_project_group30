from setuptools import find_packages, setup
from glob import glob
import os

package_name = "lab4"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "tiago_tasks_priority = lab4.tiago_tasks_priority:main",
            "tiago_tasks_priority_client = lab4.tiago_tasks_priority_client:main",
        ],
    },
)
