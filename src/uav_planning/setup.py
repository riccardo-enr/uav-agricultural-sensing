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
        # Launch files organized in subfolders
        (
            os.path.join("share", package_name, "launch"),
            ["launch/test_launch.sh"],  # Include shell scripts in root launch
        ),
        (
            os.path.join("share", package_name, "launch", "demos"),
            glob("launch/demos/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "launch", "system"),
            glob("launch/system/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "launch", "actions"),
            glob("launch/actions/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="riccardo",
    maintainer_email="riccardo.enrico97@gmail.com",
    description="UAV Planning Package - Bio-inspired path planning and trajectory generation for agricultural UAVs",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Core components
            "uav_controller = uav_planning.core.uav_controller:main",
            "uav_monitor = uav_planning.core.uav_monitor:main",
            "uav_state_machine = uav_planning.core.uav_state_machine:main",
            # Algorithm nodes
            "bioinspired_planner = uav_planning.algorithms.bioinspired_path_generator:main",
            # Main planning nodes
            "butterfly_path_node = uav_planning.nodes.uav_planning_node:main",
            # Action servers and clients
            "bioinspired_planner_action = uav_planning.actions.bioinspired_path_generator_action:main",
            "bioinspired_path_generator_client = uav_planning.actions.bioinspired_path_generator_client:main",
            "path_generator_client = uav_planning.actions.path_generator_client:main",
            # Utilities
            "architecture_test = uav_planning.utils.architecture_test:main",
            "state_machine_demo = uav_planning.demos.state_machine_demo:main",
        ],
    },
)
