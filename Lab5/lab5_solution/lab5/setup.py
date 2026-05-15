from setuptools import find_packages, setup

package_name = "lab5"

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
            "add_collision_obj = lab5.add_collision_obj:main",
            "clear_collision_obj = lab5.clear_collision_obj:main",
            "frame_listener = lab5.frame_listener:main",
        ],
    },
)
