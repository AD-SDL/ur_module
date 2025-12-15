"""Controls Various Type of Gripper End Effectors"""

import logging
import socket
import traceback
from copy import deepcopy
from math import radians
from time import sleep
from typing import Union

from ur_interface.integrated_controller import UREndEffector
from ur_interface.ur_error_types import GripperConnectionError, GripperOperationError

from .grippers.robotiq_gripper_interface import RobotiqGripper

gripper_map = {
    UREndEffector.Robotiq2FingerGripper: RobotiqGripper
}
class GripperController:
    """Controls Robotiq Finger Grippers"""

    def __init__(
        self,
        end_effector: UREndEffector = UREndEffector.Robotiq2FingerGripper,
        logger: logging.Logger = None,
        **kwargs
    ):
        """
        Constructor for the FingerGripperController class.

        Args:
            hostname (str): The hostname of the robot.
            port (int): Port number to connect to the robot over the Interpreter socket
        """
        
        self.gripper_type = end_effector
        self.gripper = gripper_map[self.gripper_type](**kwargs)
        self.connect_gripper(**kwargs)

    def __del__(self):
        """Destructor for the GripperController class."""
        self.disconnect_gripper()

    def connect_gripper(self, **kwargs) -> None:
        """
        Connect to the gripper
        """
        self.gripper.connect(**kwargs)


            

    def disconnect_gripper(self):
        """
        Disconnect from the gripper
        """
        self.gripper.disconnect()
       

    

    def open_gripper(
        self,
        **kwargs
    ) -> None:
        """Opens the gripper"""
        self.gripper.open(**kwargs)

    def close_gripper(
        self,
        **kwargs
    ) -> None:
        """Closes the gripper"""
        self.gripper.close(**kwargs)

    

