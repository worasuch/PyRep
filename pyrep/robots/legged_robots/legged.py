from pyrep.backend import sim, utils
from pyrep.objects import Object
from pyrep.objects.dummy import Dummy
from pyrep.robots.configuration_paths.arm_configuration_path import (
    ArmConfigurationPath)
from pyrep.robots.leggedrobot_component import LeggedRobotComponent
# from pyrep.robots.robot_component import RobotComponent
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.errors import ConfigurationError, ConfigurationPathError, IKError
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.const import PYREP_SCRIPT_TYPE
from typing import List, Union
import numpy as np
import warnings

"""
class Legged(RobotComponent):
    def __init__(self, count: int, name: str, joint_names: List[str], base_name: str = None):
        super().__init__(count, name, joint_names, base_name)
"""

class Legged(LeggedRobotComponent):

    def __init__(self):
        super().__init__()