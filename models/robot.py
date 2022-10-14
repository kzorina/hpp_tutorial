from typing import List
import numpy as np
from pyphysx_utils.urdf_robot_parser import URDFRobot
from hpp.corbaserver.manipulation.robot import Robot
from models.models_utils import get_models_path
import pathlib

class PandaRobot(Robot):
    main_folder = pathlib.Path(__file__).absolute().parent
    urdfFilename = str(main_folder.joinpath('franka_panda/panda.urdf'))
    srdfFilename = str(main_folder.joinpath('franka_panda/panda.srdf'))
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, compositeName="panda", robotName="panda", load=True, rootJointType="anchor"):
        Robot.__init__(self, compositeName, robotName, rootJointType, load)

    @staticmethod
    def initial_configuration() -> List[float]:
        """ Return the initial configuration of the robot. """
        return [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4, 0., 0.]

    @property
    def reach_m(self):
        """
        :return: Maximum reach of robot end effector in meters.
        """
        return 0.8

    def get_actuated_joint_names(self) -> List[str]:
        """Return list of names of actuated joints"""
        return [joint_name for joint_name in self.getAllJointNames() if self.getJointNumberDof(joint_name)]

    @staticmethod
    def get_pyphysx_robot():
        return URDFRobot(
            get_models_path().joinpath('franka_panda/panda.urdf'),
            mesh_path=get_models_path(),
            kinematic=True,
        )

    @staticmethod
    def get_gripper_name():
        return "panda/gripper"

    def modify_open_gripper(self, config):
        ndof = len(self.initial_configuration())
        config[ndof - 2:ndof] = [0.039, 0.039]
        return config