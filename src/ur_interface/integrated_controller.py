import logging
import traceback
from enum import Enum
from typing import Optional

from madsci.client.resource_client import ResourceClient
from madsci.common.types.auth_types import OwnershipInfo

from ur_interface.ur_controller import URController
from ur_interface.ur_dashboard import URDashboard
from ur_interface.ur_tools.grippers.finger_gripper_functions_mixin import FingerGripperMixin
from ur_interface.ur_tools.grippers.robotiq_2_finger_gripper_interface import Robotiq2FingerGripper


class UREndEffector(str, Enum):
    ROBOTIQ2FINGERGRIPPER = "ROBOTIQ2FINGERGRIPPER"
    SCREWDRIVER = "SCREWDRIVER"
    PIPETTE = "PIPETTE"
    WMTOOLCHANGER = "WMTOOLCHANGER"


end_effectors = {UREndEffector.ROBOTIQ2FINGERGRIPPER: Robotiq2FingerGripper}


class IntegratedController(FingerGripperMixin):
    """
    This is the primary class for UR robots.
    It integrates various interfaces to achieve comprehensive control, encompassing robot initialization via the UR dashboard,
    robot motion using URx, and the management of robot end-effectors such as grippers, screwdrivers, electronic pipettes, and cameras."
    """

    def __init__(
        self,
        hostname: str = None,
        resource_client: ResourceClient = None,
        end_effector_resource_id: str = None,
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
        self.end_effector_resource_id = end_effector_resource_id

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
