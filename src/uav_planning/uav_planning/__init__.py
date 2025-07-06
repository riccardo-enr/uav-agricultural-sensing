"""
UAV Planning Package

This package contains modules for UAV path planning and trajectory generation.
Organized into submodules:
- core: Core UAV control components
- algorithms: Path planning algorithms
- nodes: ROS 2 node implementations
- actions: ROS 2 action servers and clients
- utils: Utility functions and helper classes
"""

# Import key classes from submodules
from .algorithms.butterfly_generator import ButterflyGenerator
from .algorithms.bioinspired_path_generator import BioinspiredPathGenerator
from .algorithms.butterfly import ButterflyExplorer
from .core.uav_controller import UAVController
from .core.uav_monitor import UAVMonitor
from .nodes.uav_planning_node import ButterflyPathNode
from .actions.path_generator_client import PathGeneratorActionClient
from .utils.polygon import DeterministicCornerExplorer

__all__ = [
    "ButterflyGenerator",
    "BioinspiredPathGenerator",
    "ButterflyExplorer",
    "UAVController", 
    "UAVMonitor",
    "ButterflyPathNode",
    "PathGeneratorActionClient",
    "DeterministicCornerExplorer"
]
