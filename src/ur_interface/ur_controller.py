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

from ur_interface.ur_dashboard import UR_DASHBOARD
from ur_interface.ur_error_types import GripperError, URConnectionError, URMovementError
from ur_interface.ur_tools.gripper_controller import FingerGripperController
from ur_interface.ur_tools.ot_pipette_controller import OTPipetteController
from ur_interface.ur_tools.screwdriver_controller import ScrewdriverController
from ur_interface.ur_tools.tricontinent_pipette_controller import TricontinentPipetteController
from ur_interface.ur_tools.wm_tool_changer_controller import WMToolChangerController
    
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
            raise
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
        try:
            self.ur.disconnect_ur()
            self.ur_dashboard.clear_operational_mode()
            self.ur_dashboard.disconnect()
            self.logger.info("UR disconnected successfully")
        except Exception as e:
            self.logger.error(f"Error during UR disconnect: {e}\n{traceback.format_exc()}")

    def _set_base_reference_frame(self, base_reference_frame: list) -> None:
        """Sets the base reference frame for the robot.
        Args:
            base_reference_frame (list): 6 element reference frame. [x, y, z, rx, ry, rz]. Rotation values are in degrees, later converted to radians in the function.
        """
        if not isinstance(base_reference_frame, list) or len(base_reference_frame) != 6:
            raise ValueError("Base reference frame must be a list of 6 values")

        try:
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
        except Exception as e:
            self.logger.error(f"Error setting base reference frame: {e}\n{traceback.format_exc()}")
            raise

    def get_movement_state(self) -> str:
        """Gets robot movement status by checking robot joint values.
        Return (str) READY if robot is not moving
                     BUSY if robot is moving
        """
        try:
            current_location = self.ur_connection.getj()
            if self.robot_current_joint_angles is None:
                movement_state = "READY"
            else:
                if np.linalg.norm(np.array(current_location) - np.array(self.robot_current_joint_angles)) < 1e-3:
                    movement_state = "READY"
                else:
                    movement_state = "BUSY"

            self.robot_current_joint_angles = current_location

            return movement_state, current_location

        except Exception as e:
            self.logger.error(f"Error getting movement state: {e}\n{traceback.format_exc()}")
            raise URMovementError("Failed to get robot movement state")  # noqa

    def move_to_location(self, home_location: Union[LocationArgument, list], linear_motion: bool = False) -> None:
        """Moves the robot to the home location.

        Args: home_location: 6 joint value location
        """
        try:
            self.logger.info("Homing the robot...")
            if isinstance(home_location, LocationArgument):
                home_loc = home_location.location
            else:
                home_loc = home_location
            if linear_motion:
                self.ur_connection.movel(home_loc, self.velocity, self.acceleration)
            else:
                self.ur_connection.movej(home_loc, self.velocity, self.acceleration)
            self.logger.info("Robot homed")
        except Exception as e:
            self.logger.error(f"Error in homing the robot: {e}\n{traceback.format_exc()}")
            raise URMovementError("Failed to home the robot")  # noqa

    def pick_tool(
        self,
        home: Union[LocationArgument, list] = None,
        tool_loc: Union[LocationArgument, list] = None,
        docking_axis: str = "y",
        payload: float = 0.12,
        tool_name: str = None,
    ) -> None:
        """Picks up a tool using the given tool location
        Args
            home (Union[LocationArgument, list]): Home location
            tool_loc (Union[LocationArgument, list]): Tool location
            docking_axis (str): Docking axis (x/y/z). Default: Y AXIS
            payload (float): Weight of the end effector
            tool_name (str): Name of the tool to indentify system variables
        """

        try:
            self.ur_connection.set_payload(payload)
            wingman_tool = WMToolChangerController(
                tool_location=tool_loc,
                docking_axis=docking_axis,
                ur=self.ur_connection,
                tool=tool_name,
                resource_client=self.resource_client,
                tool_resource_id=self.tool_resource_id,
            )

            self.home(home)
            wingman_tool.pick_tool()
            if self.resource_client and isinstance(tool_loc, LocationArgument):  # Handle resources if configured
                tool_resource = self.resource_client.get_resource(tool_loc.resource_id)
                tool_resource.owner = self.resource_owner
                self.resource_client.update_resource(
                    resource_id=tool_resource.resource_id,
                )
            self.tool_resource_id = tool_resource.resource_id
            self.home(home)

        except Exception as err:
            self.logger.error(f"Error in picking tool: {err}\n{traceback.format_exc()}")

    def place_tool(
        self,
        home: Union[LocationArgument, list] = None,
        tool_loc: Union[LocationArgument, list] = None,
        docking_axis: str = "y",
        tool_name: str = None,
    ) -> None:
        """Places a tool back to tool docking location
        Args
            home (Union[LocationArgument, list]): Home location
            tool_loc (Union[LocationArgument, list]): Tool location
            docking_axis (str): Docking axis (x/y/z). Default: Y AXIS
            tool_name (str): Name of the tool to indentify system variables

        """
        try:
            wingman_tool = WMToolChangerController(
                tool_location=tool_loc,
                docking_axis=docking_axis,
                ur=self.ur_connection,
                tool=tool_name,
                resource_client=self.resource_client,
                tool_resource_id=self.tool_resource_id,
            )
            self.home(home)
            wingman_tool.place_tool()
            if self.resource_client and isinstance(tool_loc, LocationArgument):  # Handle resources if configured
                tool_resource = self.resource_client.get_resource(tool_loc.resource_id)
                tool_resource.owner = None
                self.resource_client.update_resource(
                    resource_id=tool_resource.resource_id,
                )
            self.tool_resource_id = None
            self.home(home)

        except Exception as err:
            self.logger.error(f"Error in placing tool: {err}\n{traceback.format_exc()}")

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

    def gripper_transfer(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        source_approach_axis: str = None,
        target_approach_axis: str = None,
        source_approach_distance: float = None,
        target_approach_distance: float = None,
        gripper_open: int = None,
        gripper_close: int = None,
    ) -> None:
        """Make a transfer using the finger gripper. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location
            source (Union[LocationArgument, list]): Source location
            target(Union[LocationArgument, list]): Target location
            source_approach_axis (str): Source approach axis (X/Y/Z)
            target_approach_axis (str): Target approach axis (X/Y/Z)
            source_approach_distance (float): Source approach distance. Unit meters.
            target_approach_distance(float): Target approach distance. Unit meters.
            gripper_open (int): Gripper max open value (0-255)
            gripper_close (int): Gripper min close value (0-255)

        """

        if not source or not target:
            raise ValueError("Please provide both the source and target locations to make a transfer")
        gripper_controller = None

        try:
            self.logger.info(f"Starting gripper transfer from {source} to {target}")
            self.home(home)

            self.logger.info("Initializing gripper controller")
            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.tool_resource_id,
                logger=self.logger,
            )
            self.logger.info("Connecting to gripper...")
            gripper_controller.connect_gripper()
            gripper_controller.velocity = self.velocity
            gripper_controller.acceleration = self.acceleration
            gripper_controller.gripper_speed = self.gripper_speed
            gripper_controller.gripper_force = self.gripper_force

            if gripper_open:
                gripper_controller.gripper_open = gripper_open
            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            self.logger.info("Executing gripper transfer...")
            gripper_controller.transfer(
                home=home,
                source=source,
                target=target,
                source_approach_axis=source_approach_axis,
                target_approach_axis=target_approach_axis,
                source_approach_distance=source_approach_distance,
                target_approach_distance=target_approach_distance,
            )
            self.logger.info("Gripper transfer completed successfully")
        except socket.timeout as e:
            self.logger.error(f"Socket timeout during gripper transfer: {e}\n{traceback.format_exc()}")
            raise GripperError(f"Gripper communication timed out: {e}")  # noqa

        except Exception as e:
            self.logger.error(f"Error in gripper transfer action: {e}\n{traceback.format_exc()}")
            raise GripperError(f"Gripper transfer failed: {e}")  # noqa

        finally:
            if gripper_controller:
                try:
                    self.logger.info("Disconnecting gripper...")
                    gripper_controller.disconnect_gripper()
                except Exception as e:
                    self.logger.error(f"Error disconnecting gripper: {e}\n{traceback.format_exc()}")

            try:
                self.home(home)
            except Exception as e:
                self.logger.error(f"Error returning to home after gripper transfer: {e}\n{traceback.format_exc()}")

    def gripper_pick(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        source_approach_axis: str = None,
        source_approach_distance: float = None,
        gripper_close: int = None,
    ) -> None:
        """Make a transfer using the finger gripper. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location
            source (Union[LocationArgument, list]): Source location
            source_approach_axis (str): Source approach axis (X/Y/Z)
            source_approach_distance (float): Source approach distance. Unit meters.
            gripper_close (int): Gripper min close value (0-255)

        """
        if not source:
            raise ValueError("Please provide the source location to make a pick")

        gripper_controller = None

        try:
            self.logger.info(f"Starting gripper pick from {source}")
            self.home(home)

            self.logger.info("Initializing gripper controller...")
            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.tool_resource_id,
                logger=self.logger,
            )

            self.logger.info("Connecting to gripper...")
            gripper_controller.connect_gripper()
            gripper_controller.velocity = self.velocity
            gripper_controller.acceleration = self.acceleration
            gripper_controller.gripper_speed = self.gripper_speed
            gripper_controller.gripper_force = self.gripper_force

            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            self.logger.info("Executing gripper pick...")
            gripper_controller.pick(
                source=source,
                approach_axis=source_approach_axis,
                approach_distance=source_approach_distance,
            )
            self.logger.info("Gripper pick completed successfully")

        except socket.timeout as e:
            self.logger.error(f"Socket timeout during gripper pick: {e}\n{traceback.format_exc()}")
            raise GripperError(f"Gripper communication timed out during pick: {e}")  # noqa

        except Exception as e:
            self.logger.error(f"Error in gripper pick action: {e}\n{traceback.format_exc()}")
            raise GripperError(f"Gripper pick failed: {e}")  # noqa

        finally:
            if gripper_controller:
                try:
                    self.logger.info("Disconnecting gripper...")
                    gripper_controller.disconnect_gripper()
                except Exception as e:
                    self.logger.error(f"Error disconnecting gripper: {e}\n{traceback.format_exc()}")

            try:
                self.home(home)
            except Exception as e:
                self.logger.error(f"Error returning to home after gripper pick: {e}\n{traceback.format_exc()}")

    def gripper_place(
        self,
        home: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        target_approach_axis: str = None,
        target_approach_distance: float = None,
        gripper_open: int = None,
    ) -> None:
        """Make a transfer using the finger gripper. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location
            target (Union[LocationArgument, list]): Source location
            target_approach_axis (str): Source approach axis (X/Y/Z)
            target_approach_distance (float): Source approach distance. Unit meters.
            gripper_open (int): Gripper max open value (0-255)

        """

        if not target:
            raise ValueError("Please provide the target location to make a place")

        gripper_controller = None

        try:
            self.logger.info(f"Starting gripper place to {target}")
            self.home(home)

            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.tool_resource_id,
                logger=self.logger,
            )

            self.logger.info("Connecting to gripper...")
            gripper_controller.connect_gripper()
            gripper_controller.velocity = self.velocity
            gripper_controller.acceleration = self.acceleration
            gripper_controller.gripper_speed = self.gripper_speed
            gripper_controller.gripper_force = self.gripper_force

            if gripper_open:
                gripper_controller.gripper_open = gripper_open

            self.logger.info("Executing gripper place...")
            gripper_controller.place(
                target=target,
                approach_axis=target_approach_axis,
                approach_distance=target_approach_distance,
            )
            self.logger.info("Gripper place completed successfully")

        except socket.timeout as e:
            self.logger.error(f"Socket timeout during gripper place: {e}\n{traceback.format_exc()}")
            raise GripperError(f"Gripper communication timed out during place: {e}")  # noqa

        except Exception as e:
            self.logger.error(f"Error in gripper place action: {e}\n{traceback.format_exc()}")
            raise GripperError(f"Gripper place failed: {e}")  # noqa

        finally:
            if gripper_controller:
                try:
                    self.logger.info("Disconnecting gripper...")
                    gripper_controller.disconnect_gripper()
                except Exception as e:
                    self.logger.error(f"Error disconnecting gripper: {e}\n{traceback.format_exc()}")

            try:
                self.home(home)
            except Exception as e:
                self.logger.error(f"Error returning to home after gripper place: {e}\n{traceback.format_exc()}")

    def gripper_screw_transfer(
        self,
        home: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        screwdriver_loc: Union[LocationArgument, list] = None,
        screw_loc: Union[LocationArgument, list] = None,
        screw_time: float = 9,
        gripper_open: int = None,
        gripper_close: int = None,
    ) -> None:
        """Using custom made screwdriving solution. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location
            target(Union[LocationArgument, list]): Target location
            screwdriver_loc (Union[LocationArgument, list]): Location of the screwdriving bit
            screw_loc (Union[LocationArgument, list]): Location where the screwdriving will be performed
            screw_time (float): Screwdriving duration
            gripper_open (int): Gripper max open value (0-255)
            gripper_close (int): Gripper min close value (0-255)

        """

        self.home(home)

        try:
            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.tool_resource_id,
            )

            gripper_controller.connect_gripper()

            if gripper_open:
                gripper_controller.gripper_open = gripper_open
            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            gripper_controller.screw_transfer(
                home=home, target=target, screw_loc=screw_loc, screwdriver_loc=screwdriver_loc, screw_time=screw_time
            )

        except Exception as err:
            self.logger.error(
                f"Error during gripper screw transfer: {err}\n{traceback.format_exc()}"
            )  # Added exc_info=True for detailed logging

        finally:
            gripper_controller.disconnect_gripper()

    def remove_cap(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        gripper_open: int = None,
        gripper_close: int = None,
    ) -> None:
        """Remove vial cap. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location
            source (Union[LocationArgument, list]): Source location
            target (Union[LocationArgument, list]): Target location
            gripper_open (int): Gripper max open value (0-255)
            gripper_close (int): Gripper min close value (0-255)

        """
        self.home(home)

        try:
            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.tool_resource_id,
            )

            gripper_controller.connect_gripper()
            if gripper_open:
                gripper_controller.gripper_open = gripper_open
            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            gripper_controller.remove_cap(home=home, target=target, source=source)
            gripper_controller.disconnect_gripper()

        except Exception as err:
            self.logger.error(f"{err}\n{traceback.format_exc()}")

    def place_cap(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        gripper_open: int = None,
        gripper_close: int = None,
    ) -> None:
        """Place vial cap. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location
            source (Union[LocationArgument, list]): Source location
            target (Union[LocationArgument, list]): Target location
            gripper_open (int): Gripper max open value (0-255)
            gripper_close (int): Gripper min close value (0-255)

        """
        self.home(home)

        try:
            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.tool_resource_id,
            )
            gripper_controller.place_cap(home=home, target=target, source=source)
            gripper_controller.disconnect_gripper()

        except Exception as err:
            self.logger.error(f"{err}\n{traceback.format_exc()}")

    def pick_and_flip_object(
        self,
        home: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        approach_axis: str = None,
        gripper_open: int = None,
        gripper_close: int = None,
    ) -> None:
        """
        Pick an object then flips it and puts it back to the same location. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location
            target (Union[LocationArgument, list]): Target location
            approach_axis (str) = Object approach axis
            gripper_open (int): Gripper max open value (0-255)
            gripper_close (int): Gripper min close value (0-255)

        """

        self.home(home)

        try:
            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.gripper_resource_id,
            )

            gripper_controller.connect_gripper()

            if gripper_open:
                gripper_controller.gripper_open = gripper_open
            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            gripper_controller.flip_object(target=target, approach_axis=approach_axis)
            gripper_controller.disconnect_gripper()
            self.home(home)
        except Exception as er:
            self.logger.error(er)
        finally:
            gripper_controller.disconnect_gripper()

    def robotiq_screwdriver_transfer(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        source_approach_axis: str = None,
        target_approach_axis: str = None,
        source_approach_distance: float = None,
        target_approach_distance: float = None,
    ) -> None:
        """
        Make a screw transfer using the Robotiq Screwdriver. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location
            source (Union[LocationArgument, list]): Source location
            target(Union[LocationArgument, list]): Target location
            source_approach_axis (str): Source approach axis (X/Y/Z)
            target_approach_axis (str): Target approach axis (X/Y/Z)
            source_approach_distance (float): Source approach distance. Unit meters.
            target_approach_distance(float): Target approach distance. Unit meters.
        """
        self.home(home)

        try:
            sr = ScrewdriverController(
                hostname=self.hostname,
                ur=self.ur_connection,
                ur_dashboard=self.ur_dashboard,
            )
            sr.screwdriver.activate_screwdriver()
            sr.transfer(
                source=source,
                target=target,
                source_approach_axis=source_approach_axis,
                target_approach_axis=target_approach_axis,
                source_approach_dist=source_approach_distance,
                target_approach_dist=target_approach_distance,
            )
            sr.screwdriver.disconnect()
        except Exception as err:
            self.logger.error(err)

        self.home(home)

    def pipette_transfer(
        self,
        home: Union[LocationArgument, list] = None,
        tip_loc: Union[LocationArgument, list] = None,
        tip_trash: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        volume: int = 10,
        pipette_speed: int = 150,
    ) -> None:
        """
        Make a liquid transfer using the pipette. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location joint values
            tip_loc (Union[LocationArgument, list]): Pipette tip location
            tip_trash (Union[LocationArgument, list]): Tip trash location
            source (str): Source location
            target (str): Target location
            volume (int): Pipette transfer volume. Unit number of steps. Each step is 1 mL
        """
        if not source or not target:
            raise Exception("Please provide both the source and target loactions to make a transfer")

        try:
            pipette = TricontinentPipetteController(
                hostname=self.hostname,
                ur=self.ur_connection,
                pipette_ip=self.hostname,
                resource_client=self.resource_client,
                pipette_resource_id=self.tool_resource_id,
            )
            pipette.connect_pipette()
            if tip_loc:
                pipette.pick_tip(tip_loc=tip_loc)
            self.home(home)
            pipette.transfer_sample(
                home=home,
                sample_aspirate=source,
                sample_dispense=target,
                volume=volume,
                speed=pipette_speed,
            )
            if tip_trash:
                pipette.eject_tip(eject_tip_loc=tip_trash, approach_axis="y")
            pipette.disconnect_pipette()
            self.logger.info("Disconnecting from the pipette")
        except Exception as err:
            self.logger.error(err)

    def pipette_pick_and_move_sample(
        self,
        home: Union[LocationArgument, list] = None,
        linear_motion: bool = False,
        safe_waypoint: Union[LocationArgument, list] = None,
        tip_loc: Union[LocationArgument, list] = None,
        sample_loc: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        volume: int = 10,
        pipette_speed: int = 150,
    ) -> None:
        """Pipette pick sample from the source location and transfer it to the target location

        Args
            home (Union[LocationArgument, list]): Home location use Linear motions if needed
            safe_waypoint (Union[LocationArgument, list]): Safe waypoint location to move the pipette
            tip_loc (Union[LocationArgument, list]): Pipette tip location
            sample_loc (Union[LocationArgument, list]): Sample location
            target (Union[LocationArgument, list]): Target location
            volume (int): Pipette transfer volume. Unit number of steps. Each step is 1 mL
        """
        if not sample_loc or not target:
            raise Exception("Please provide both the sample and target loactions to make a transfer")

        try:
            pipette = TricontinentPipetteController(
                hostname=self.hostname,
                ur=self.ur_connection,
                pipette_ip=self.hostname,
                resource_client=self.resource_client,
                pipette_resource_id=self.tool_resource_id,
            )
            pipette.connect_pipette(speed=pipette_speed)
            pipette.initialize_pipette()
            if tip_loc:
                pipette.pick_tip(tip_loc=tip_loc)
            if home:
                self.home(home, linear_motion=linear_motion)
            pipette.pick_and_move(
                safe_waypoint=safe_waypoint,
                sample_loc=sample_loc,
                target=target,
                volume=volume,
            )
            pipette.disconnect_pipette()
            self.logger.info("Disconnecting from the pipette")
        except Exception as err:
            self.logger.error(err)

    def pipette_dispense_and_retrieve(
        self,
        home: Union[LocationArgument, list] = None,
        linear_motion: bool = False,
        safe_waypoint: Union[LocationArgument, list] = None,
        tip_trash: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        volume: int = 10,
        pipette_speed: int = 150,
    ) -> None:
        """Dispense a sample using the pipette. This function uses linear motions to perform the pick and place movements.
        Args
            home (Union[LocationArgument, list]): Home location joint values
            tip_trash (Union[LocationArgument, list]): Tip trash location
            target (Union[LocationArgument, list]): Target location
            volume (int): Pipette transfer volume. Unit number of steps. Each step is 1 mL
        """
        if not target:
            raise Exception("Please provide the target loaction to make a dispense")

        try:
            pipette = TricontinentPipetteController(
                hostname=self.hostname,
                ur=self.ur_connection,
                pipette_ip=self.hostname,
                resource_client=self.resource_client,
                pipette_resource_id=self.tool_resource_id,
            )
            pipette.connect_pipette(speed=pipette_speed)
            pipette.dispense_and_retrieve(
                target=target,
                safe_waypoint=safe_waypoint,
                volume=volume,
            )
            if tip_trash:
                pipette.eject_tip(eject_tip_loc=tip_trash, approach_axis="y")
            pipette.disconnect_pipette()
            self.home(home, linear_motion=linear_motion)
            self.logger.info("Disconnecting from the pipette")
        except Exception as err:
            self.logger.error(err)

    def run_droplet(
        self,
        home: Union[LocationArgument, list] = None,
        tip_loc: Union[LocationArgument, list] = None,
        sample_loc: Union[LocationArgument, list] = None,
        droplet_loc: Union[LocationArgument, list] = None,
        tip_trash: Union[LocationArgument, list] = None,
    ) -> None:
        """Run the full droplet protocol cycle

        Args
            home (Union[LocationArgument, list]): Home location
            tip_loc (Union[LocationArgument, list]): Pipette tip location
            sample_loc (Union[LocationArgument, list]): Sample location
            droplet_loc (Union[LocationArgument, list]): Location where the droplet will be hung
            tip_trash (Union[LocationArgument, list]): Pipette tip trash location
        """
        pipette = OTPipetteController(ur_connection=self.ur_connection, IP=self.hostname)
        pipette.connect_pipette()

        self.home(home)
        pipette.pick_tip(tip_loc=tip_loc)
        pipette.transfer_sample(sample_loc=sample_loc)
        self.home(home)
        pipette.create_droplet(droplet_loc=droplet_loc)
        self.home(home)
        pipette.empty_tip(sample_loc=sample_loc)
        pipette.eject_tip(eject_tip_loc=tip_trash)
        self.home(home)
        pipette.disconnect_pipette()

    def run_urp_program(self, transfer_file_path: str = None, program_name: str = None):
        """Transfers the urp programs onto the polyscope and initiates them

        Args:
            trasnfer_file_path (str): Local file path
            program_name (str): Name of the file
        """
        if not program_name:
            raise ValueError("Provide program name!")

        ur_program_path = "/programs/" + program_name

        if transfer_file_path:
            self.ur_dashboard.transfer_program(local_path=transfer_file_path, remote_path=ur_program_path)
            sleep(2)

        self.ur_dashboard.load_program(program_path=ur_program_path)
        sleep(2)
        self.ur_dashboard.run_program()
        sleep(5)

        self.logger.info(f"Running the URP program: {program_name}")
        time_elapsed = 0

        program_status = "BUSY"
        ready_status_count = 0
        while program_status == "BUSY":
            if self.get_movement_state() == "READY":
                ready_status_count += 1
                if ready_status_count >= 6:
                    program_status = "READY"
            else:
                ready_status_count = 0
            sleep(3)

        program_log = {
            "output_code": "0",
            "output_msg": "Successfully finished " + program_name,
            "output_log": "seconds_elapsed:" + str(time_elapsed),
        }

        return program_log
