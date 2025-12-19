#!/usr/bin/env python3
"""Interface for UR Driver"""

import logging
import socket
import traceback
from math import radians
from time import sleep
from typing import Optional, Union

import math3d as m3
import numpy as np
from madsci.client.resource_client import ResourceClient
from madsci.common.types.auth_types import OwnershipInfo
from madsci.common.types.location_types import LocationArgument
from urx import Robot

from ur_interface.ur_error_types import URConnectionError


class URController:
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
        self.gripper_speed: int = None
        self.gripper_force: int = None

        try:
            self.ur_connection = self.connect_ur(hostname=self.hostname, logger=self.logger)
            self.ur_connection.set_tcp(tcp_pose)
            if base_reference_frame:
                self._set_base_reference_frame(base_reference_frame)
            self.get_movement_state()

        except Exception as e:
            self.logger.error(f"Failed to initialize UR: {e}\n{traceback.format_exc()}")
            raise e

    def connect_ur(self, hostname: str = None) -> Robot:
        """Create connection to the UR robot"""
        for attempt in range(10):
            try:
                self.logger.info(f"Attempting robot connection (attempt {attempt + 1}/10)...")
                robot_connection = Robot(self.hostname)

            except socket.error as e:
                self.logger.warning(f"Robot connection attempt {attempt + 1} failed: {e}")
                sleep(1)

            except Exception as e:
                self.logger.error(f"Unexpected error during robot connection: {e}\n{traceback.format_exc()}")
                sleep(1)

            else:
                self.logger.info("Successful UR connection")
                return robot_connection

        raise URConnectionError(f"Failed to connect to UR robot at {self.hostname} after 10 attempts")

    def disconnect_ur(self):
        """
        Description: Disconnects the socket connection with the UR robot
        """
        try:
            if self.ur_connection:
                self.ur_connection.close()
                self.logger.info("Robot connection closed successfully")
        except Exception as e:
            self.logger.error(f"Error closing robot connection: {e}\n{traceback.format_exc()}")
            raise e

    def _setup_logger(self) -> logging.Logger:
        """Setup default logger if none provided"""
        logger = logging.getLogger(__name__)
        logger.setLevel(logging.INFO)
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        return logger

    def disconnect(self):
        """Disconnects the robot from URX and UR Dashboard connections"""
        self.ur.disconnect_ur()
        self.ur_dashboard.clear_operational_mode()
        self.ur_dashboard.disconnect()
        self.logger.info("UR disconnected successfully")

    def _set_base_reference_frame(self, base_reference_frame: list) -> None:
        """Sets the base reference frame for the robot.
        Args:
            base_reference_frame (list): 6 element reference frame. [x, y, z, rx, ry, rz]. Rotation values are in degrees, later converted to radians in the function.
        """
        if not isinstance(base_reference_frame, list) or len(base_reference_frame) != 6:
            raise ValueError("Base reference frame must be a list of 6 values")

        # Extract position and rotation components
        x, y, z, rx_deg, ry_deg, rz_deg = base_reference_frame

        # Create translation vector (only if any translation values are non-zero)
        if any([x, y, z]):
            translation = m3.Vector(x, y, z)
        else:
            translation = m3.Vector(0, 0, 0)

        # Start with identity rotation
        rotation = m3.Orientation()  # Identity rotation

        # Apply only non-zero rotations in order
        if rx_deg != 0:
            rx_rad = radians(rx_deg)
            rotation = rotation * m3.Orientation.new_rot_x(rx_rad)

        if ry_deg != 0:
            ry_rad = radians(ry_deg)
            rotation = rotation * m3.Orientation.new_rot_y(ry_rad)

        if rz_deg != 0:
            rz_rad = radians(rz_deg)
            rotation = rotation * m3.Orientation.new_rot_z(rz_rad)
        # Create the transform
        transform = m3.Transform(rotation, translation)

        # Set the coordinate system
        self.ur_connection.set_csys(transform)
        self.logger.info(f"Base reference frame set to: {base_reference_frame}")

    def get_movement_state(self) -> str:
        """Gets robot movement status by checking robot joint values.
        Return (str) READY if robot is not moving
                     BUSY if robot is moving
        """
        current_location = {"joint_angles": self.ur_connection.getj(), "linear_coordinates": self.ur_connection.getl()}
        if self.robot_current_joint_angles is None:
            movement_state = "READY"
        else:
            if (
                np.linalg.norm(np.array(current_location["joint_angles"]) - np.array(self.robot_current_joint_angles))
                < 1e-3
            ):
                movement_state = "READY"
            else:
                movement_state = "BUSY"

        self.robot_current_joint_angles = current_location["joint_angles"]

        return movement_state, current_location

    def move_to_location(self, location: Union[LocationArgument, list], linear_motion: bool = False) -> None:
        """Moves the robot to the home location.

        Args: home_location: 6 joint value location
        """
        try:
            self.logger.info("Homing the robot...")

            if linear_motion:
                if isinstance(location, LocationArgument):
                    location = location.representation.linear_coordinates
                self.ur_connection.movel(location, self.velocity, self.acceleration)
            else:
                if isinstance(location, LocationArgument):
                    location = location.representation.joint_angles
                self.ur_connection.movej(location, self.velocity, self.acceleration)
            self.logger.info("Robot moved")
        except Exception as e:
            self.logger.error(f"Error in moving the robot: {e}\n{traceback.format_exc()}")
            raise e

    def set_digital_io(self, channel: int = None, value: bool = None) -> None:
        """Sets digital I/O outputs to open an close the channel. This helps controlling the external tools

        Args
            channel (int): Channel number
            value (bool): False for close, True for open
        """
        if channel is None or value is None:
            self.logger.error("Channel or value is not specified")
            return
        self.ur_connection.set_digital_out(channel, value)
