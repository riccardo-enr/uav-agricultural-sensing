from setuptools import find_packages, setup
import os
from glob import glob

package_name = "uav_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="riccardo",
    maintainer_email="riccardo.enrico97@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bioinspired_planner = uav_planning.bioinspired_path_generator:main",
            "bioinspired_planner_action = uav_planning.bioinspired_path_generator_action:main",
            "path_generator_action = uav_planning.path_generator_action:main",
            "uav_controller = uav_planning.uav_controller:main",
            "path_generator_client = uav_planning.path_generator_client:main",
            "architecture_test = uav_planning.architecture_test:main",
            "uav_monitor = uav_planning.uav_monitor:main",
        ],
    },
)
