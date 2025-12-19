import logging
import traceback
from copy import deepcopy
from enum import Enum
from typing import Optional, Union

from madsci.client.resource_client import ResourceClient
from madsci.common.types.auth_types import OwnershipInfo
from madsci.common.types.location_types import LocationArgument

from ur_interface.ur_controller import URController
from ur_interface.ur_dashboard import URDashboard
from ur_interface.ur_tools.grippers.abstract_gripper_interface import Gripper
from ur_interface.ur_tools.grippers.robotiq_2_finger_gripper_interface import Robotiq2FingerGripper


class UREndEffector(str, Enum):
    ROBOTIQ2FINGERGRIPPER = "ROBOTIQ2FINGERGRIPPER"
    SCREWDRIVER = "SCREWDRIVER"
    PIPETTE = "PIPETTE"
    WMTOOLCHANGER = "WMTOOLCHANGER"


end_effectors = {UREndEffector.ROBOTIQ2FINGERGRIPPER: Robotiq2FingerGripper}


class IntegratedController:
    """
    This is the primary class for UR robots.
    It integrates various interfaces to achieve comprehensive control, encompassing robot initialization via the UR dashboard,
    robot motion using URx, and the management of robot end-effectors such as grippers, screwdrivers, electronic pipettes, and cameras."
    """

    def __init__(
        self,
        hostname: str = None,
        resource_client: ResourceClient = None,
        resource_owner: OwnershipInfo = None,
        tool_resource_id: str = None,
        tcp_pose: list = [0, 0, 0, 0, 0, 0],
        base_reference_frame: list = None,
        logger: Optional[logging.Logger] = None,
    ):
        """Constructor for the UR class.
        :param hostname: Hostname or ip.
        :param logger: Logger object for logging messages
        """

        if not hostname:
            raise TypeError("Hostname cannot be None Type!")

        self.hostname = hostname
        self.resource_client = resource_client
        self.tool_resource_id = tool_resource_id
        self.resource_owner = resource_owner
        self.logger = logger or self._setup_logger()

        self.acceleration = 0.5
        self.velocity = 0.5
        self.robot_current_joint_angles = None

        try:
            self.ur_dashboard = URDashboard(hostname=self.hostname)
            self.ur_controller = URController(
                hostname=self.hostname, logger=self.logger, tcp_pose=tcp_pose, base_reference_frame=base_reference_frame
            )

            self.ur_controller.ur_connection.set_tcp(tcp_pose)
            self.create_end_effector_controller()
        except Exception as e:
            self.logger.error(f"Failed to initialize UR: {e}\n{traceback.format_exc()}")
            raise e

    def create_end_effector_controller(self) -> None:
        """Create appropriate end-effector controller."""
        self.logger.log_info("Creating Robotiq 2-finger gripper controller")
        if self.end_effector.tool_params:
            self.ur_controller.ur_connection.set_tool_communication(**self.end_effector.tool_params)
        self.end_effector = end_effectors[self.end_effector](hostname=self.hostname)

    def gripper_pick(
        self,
        source: Union[LocationArgument, list],
        home: Union[LocationArgument, list, None] = None,
        approach_axis: str = None,
        approach_distance: float = None,
    ):
        """Pick up from source position"""
        if not isinstance(self.end_effector, Gripper):
            raise Exception("End-effector is not a gripper, cannot perform pick operation")
        if isinstance(source, LocationArgument):
            source_location = source.representation.linear_coordinates
        elif isinstance(source, list):
            source_location = source
        else:
            raise Exception("Please provide an appropriate source location")
        if home is not None:
            if isinstance(home, LocationArgument):
                home_location = home.representation.joint_angles
            elif isinstance(home, list):
                home_location = home
            else:
                raise Exception("Please provide an appropriate source location")
            self.ur_controller.move_to_location(home_location, linear_motion=True)

        if not approach_distance:
            approach_distance = 0.05

        axis = None

        if not approach_axis or approach_axis.lower() == "z":
            axis = 2
        elif approach_axis.lower() == "y":
            axis = 1
        elif approach_axis.lower() == "-y":
            axis = 1
            approach_distance = -approach_distance
        elif approach_axis.lower() == "x":
            axis = 0
        elif approach_axis.lower() == "-x":
            axis = 0
            approach_distance = -approach_distance

        above_goal = deepcopy(source_location)
        above_goal[axis] += approach_distance

        self.logger.info(f"Starting pick operation from source: {source_location}")

        self.end_effector.open()

        self.logger.debug("Moving to above goal position")
        self.ur_controller.move_to_location(above_goal, linear_motion=True)

        self.logger.debug("Moving to goal position")
        self.ur_controller.move_to_location(source_location, linear_motion=True)

        self.end_effector.close()

        self.logger.debug("Moving back to above goal position")
        self.ur_controller.move_to_location(above_goal, linear_motion=True)
        self.logger.info("Pick operation completed successfully")

    def gripper_place(
        self,
        target: Union[LocationArgument, list],
        home: Union[LocationArgument, list, None] = None,
        approach_axis: str = None,
        approach_distance: float = None,
    ):
        """Pick up from source position"""
        if not isinstance(self.end_effector, Gripper):
            raise Exception("End-effector is not a gripper, cannot perform place operation")
        if isinstance(target, LocationArgument):
            target_location = target.representation.linear_coordinates
        elif isinstance(target, list):
            target_location = target
        else:
            raise Exception("Please provide an appropriate source location")
        if home is not None:
            if isinstance(home, LocationArgument):
                home_location = home.representation.joint_angles
            elif isinstance(home, list):
                home_location = home
            else:
                raise Exception("Please provide an appropriate source location")
            self.ur_controller.move_to_location(above_goal, linear_motion=True)

        if not approach_distance:
            approach_distance = 0.05

        axis = None

        if not approach_axis or approach_axis.lower() == "z":
            axis = 2
        elif approach_axis.lower() == "y":
            axis = 1
        elif approach_axis.lower() == "-y":
            axis = 1
            approach_distance = -approach_distance
        elif approach_axis.lower() == "x":
            axis = 0
        elif approach_axis.lower() == "-x":
            axis = 0
            approach_distance = -approach_distance

        above_goal = deepcopy(target_location)
        above_goal[axis] += approach_distance

        self.logger.info(f"Starting pick operation from source: {target_location}")

        self.end_effector.open()

        self.logger.debug("Moving to above goal position")
        self.ur_controller.move_to_location(above_goal, linear_motion=True)

        self.logger.debug("Moving to goal position")
        self.ur_controller.move_to_location(target_location, linear_motion=True)

        self.end_effector.close()

        self.logger.debug("Moving back to above goal position")
        self.ur_controller.move_to_location(above_goal, linear_motion=True)
        self.logger.info("Pick operation completed successfully")

    def gripper_transfer(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        source_approach_axis: str = None,
        target_approach_axis: str = None,
        source_approach_distance: float = None,
        target_approach_distance: float = None,
    ) -> None:
        """Handles the transfer request"""
        self.logger.info("Starting transfer operation")
        self.pick(
            source=source,
            home=home,
            approach_axis=source_approach_axis,
            approach_distance=source_approach_distance,
        )
        self.logger.info("Pick completed")

        self.place(
            target=target,
            home=home,
            approach_axis=target_approach_axis,
            approach_distance=target_approach_distance,
        )
        self.logger.info("Place completed")
