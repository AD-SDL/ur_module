"""Controls Various Type of Gripper End Effectors"""

import logging
import socket
import traceback
from copy import deepcopy
from math import radians
from time import sleep
from typing import Union

from madsci.common.types.location_types import LocationArgument

from ur_interface.ur_error_types import GripperConnectionError, GripperOperationError

from .robotiq_gripper_driver import RobotiqGripper


class FingerGripperController:
    """Controls Robotiq Finger Grippers"""

    def __init__(
        self,
        hostname: str = "146.137.240.38",
        port: int = 63352,
        ur=None,
        resource_client=None,
        gripper_resource_id: str = None,
        logger: logging.Logger = None,
    ):
        """
        Constructor for the FingerGripperController class.

        Args:
            hostname (str): The hostname of the robot.
            port (int): Port number to connect to the robot over the Interpreter socket
        """
        self.host = hostname
        self.PORT = port
        self.resource_client = resource_client
        self.gripper_resource_id = gripper_resource_id
        self.logger = logger

        if not ur:
            raise Exception("UR connection is not established")
        else:
            self.ur = ur
            # self.ur.set_payload(1.2)# TODO: Check the actual payload

        self.gripper_close = 255  # 0-255 (255 is closed)
        self.gripper_open = 0
        self.gripper_speed = 255  # 0-255
        self.gripper_force = 255  # 0-255

        self.acceleration = 0.7
        self.velocity = 0.7
        self.speed_ms = 0.750
        self.speed_rads = 0.750
        self.accel_mss = 1.200
        self.accel_radss = 1.200
        self.blend_radius_m = 0.001
        self.ref_frame = [0, 0, 0, 0, 0, 0]

    def connect_gripper(self, max_retries: int = 2):
        """
        Connect to the gripper
        """
        for attempt in range(max_retries):
            try:
                # GRIPPER SETUP:
                self.logger.info(f"Connecting to gripper (attempt {attempt + 1}/{max_retries})...")
                self.gripper = RobotiqGripper()

                self.logger.debug(f"Attempting socket connection to {self.host}:{self.PORT}")
                self.gripper.connect(hostname=self.host, port=self.PORT, socket_timeout=5)

                if self.gripper.is_active():
                    self.logger.info("Gripper already active")
                else:
                    self.logger.info("Activating gripper...")
                    self.gripper.activate()
                    self.logger.info("Opening gripper...")
                    self.open_gripper()

                self.logger.info("Gripper is ready")
                return

            except socket.timeout as e:
                self.logger.error(f"Socket timeout on attempt {attempt + 1}: {e}")
                if attempt < max_retries - 1:
                    self.logger.info("Attempting to reset tool communication...")
                    try:
                        self.ur.set_tool_communication(
                            baud_rate=115200,
                            parity=0,
                            stop_bits=1,
                            rx_idle_chars=1.5,
                            tx_idle_chars=3.5,
                        )
                        sleep(4)
                    except Exception as reset_error:
                        self.logger.error(
                            f"Error resetting tool communication: {reset_error}\n{traceback.format_exc()}"
                        )

            except socket.error as e:
                self.logger.error(f"Socket error on attempt {attempt + 1}: {e}\n{traceback.format_exc()}")
                if attempt < max_retries - 1:
                    self.logger.info("Retrying connection...")
                    sleep(2)

            except Exception as e:
                self.logger.error(
                    f"Unexpected error connecting to gripper on attempt {attempt + 1}: {e}\n{traceback.format_exc()}"
                )
                if attempt < max_retries - 1:
                    sleep(2)

            raise GripperConnectionError(f"Failed to connect to gripper after {max_retries} attempts")  # noqa

    def disconnect_gripper(self):
        """
        Discconect from the gripper
        """
        try:
            if self.gripper:
                self.logger.info("Disconnecting gripper...")
                self.gripper.disconnect()
                self.logger.info("Gripper connection closed")
        except Exception as e:
            self.logger.error(f"Error disconnecting gripper: {e}\n{traceback.format_exc()}")
            raise

    def home_robot(self, home: Union[LocationArgument, list] = None) -> None:
        """
        Home the robot
        """
        if not home:
            return
        try:
            self.logger.info("Homing robot to specified position...")
            if isinstance(home, LocationArgument):
                home_location = home.representation
            elif isinstance(home, list):
                home_location = home
            else:
                raise Exception("Please provide an accurate home location")

            self.logger.debug(f"Homing robot to: {home_location}")
            self.ur.movej(home_location, self.acceleration, self.velocity)
        except Exception as e:
            self.logger.error(f"Error homing robot: {e}\n{traceback.format_exc()}")
            raise

    def open_gripper(
        self,
        pose: float = None,
        speed: float = None,
        force: float = None,
    ) -> None:
        """Opens the gripper using pose, speed and force variables"""
        try:
            if pose:
                self.gripper_open = pose
            if force:
                self.gripper_force = force
            if speed:
                self.gripper_speed = speed

            self.logger.info(f"Opening gripper to position: {self.gripper_open}")

            self.gripper.move_and_wait_for_pos(
                self.gripper_open,
                self.gripper_speed,
                self.gripper_force,
            )
            sleep(0.5)
            self.logger.debug("Gripper opened successfully")

        except socket.timeout as e:
            self.logger.error(f"Timeout while opening gripper: {e}")
            raise GripperOperationError(f"Gripper open operation timed out: {e}")  # noqa

        except Exception as e:
            self.logger.error(f"Error opening gripper: {e}\n{traceback.format_exc()}")
            raise GripperOperationError(f"Failed to open gripper: {e}")  # noqa

    def close_gripper(
        self,
        pose: float = None,
        speed: float = None,
        force: float = None,
    ) -> None:
        """Closes the gripper using pose, speed and force variables"""
        try:
            if pose:
                self.gripper_close = pose
            if force:
                self.gripper_force = force
            if speed:
                self.gripper_speed = speed
            self.logger.info(f"Closing gripper to position: {self.gripper_close}")

            self.gripper.move_and_wait_for_pos(
                self.gripper_close,
                self.gripper_speed,
                self.gripper_force,
            )
            sleep(0.5)
            self.logger.debug("Gripper closed successfully")

        except socket.timeout as e:
            self.logger.error(f"Timeout while closing gripper: {e}")
            raise GripperOperationError(f"Gripper close operation timed out: {e}")  # noqa

        except Exception as e:
            self.logger.error(f"Error closing gripper: {e}\n{traceback.format_exc()}")
            raise GripperOperationError(f"Failed to close gripper: {e}")  # noqa

    def pick(
        self,
        source: Union[LocationArgument, list] = None,
        approach_axis: str = None,
        approach_distance: float = None,
    ):
        """Pick up from first goal position"""
        try:
            if isinstance(source, LocationArgument):
                source_location = source.representation
            elif isinstance(source, list):
                source_location = source
            else:
                raise Exception("Please provide an accurate source location")

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

            self.open_gripper()

            self.logger.debug("Moving to above goal position")
            self.ur.movel(above_goal, self.acceleration, self.velocity)

            self.logger.debug("Moving to goal position")
            self.ur.movel(source_location, self.acceleration, self.velocity)

            self.close_gripper()

            if self.resource_client and isinstance(source, LocationArgument):  # Handle resources if configured
                try:
                    popped_object, updated_resource = self.resource_client.pop(resource=source.resource_id)
                    self.resource_client.push(resource=self.gripper_resource_id, child=popped_object)
                except Exception as e:
                    self.logger.error(f"Resource client error during pick: {e}\n{traceback.format_exc()}")

            self.logger.debug("Moving back to above goal position")
            self.ur.movel(above_goal, self.acceleration, self.velocity)
            self.logger.info("Pick operation completed successfully")

        except GripperOperationError:
            raise
        except Exception as e:
            self.logger.error(f"Error during pick operation: {e}\n{traceback.format_exc()}")
            raise GripperOperationError(f"Pick operation failed: {e}")  # noqa

    def pick_screw(
        self,
        screw_loc: Union[LocationArgument, list] = None,
    ) -> None:
        """Handles the pick screw request"""

        if isinstance(screw_loc, LocationArgument):
            source_location = screw_loc.representation
        elif isinstance(screw_loc, list):
            source_location = screw_loc

        above_goal = deepcopy(source_location)
        above_goal[2] += 0.06
        self.ur.movel(above_goal, self.acceleration, self.velocity)
        self.ur.movel(source_location, 0.2, 0.2)
        self.ur.movel(above_goal, self.acceleration, self.velocity)

    def place(
        self,
        target: Union[LocationArgument, list] = None,
        approach_axis: str = None,
        approach_distance: float = None,
    ):
        """Place down at second goal position"""
        try:
            if isinstance(target, LocationArgument):
                target_location = target.representation
            elif isinstance(target, list):
                target_location = target
            else:
                raise ValueError("Please provide an accurate target location")

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

            self.logger.info(f"Starting place operation to target: {target_location}")
            self.logger.debug("Moving to above goal position")
            self.ur.movel(above_goal, self.acceleration, self.velocity)

            self.logger.debug("Moving to goal position")
            self.ur.movel(target_location, self.acceleration, self.velocity)

            self.open_gripper()

            if self.resource_client and isinstance(target, LocationArgument):  # Handle resources if configured
                try:
                    popped_object, updated_resource = self.resource_client.pop(resource=self.gripper_resource_id)
                    self.resource_client.push(resource=target.resource_id, child=popped_object)
                except Exception as e:
                    self.logger.error(f"Resource client error during place: {e}\n{traceback.format_exc()}")

            self.logger.debug("Moving back to above goal position")
            self.ur.movel(above_goal, self.acceleration, self.velocity)
            self.logger.info("Place operation completed successfully")

        except GripperOperationError:
            raise
        except Exception as e:
            self.logger.error(f"Error during place operation: {e}\n{traceback.format_exc()}")
            raise GripperOperationError(f"Place operation failed: {e}")  # noqa

    def place_screw(
        self,
        target: Union[LocationArgument, list] = None,
        screw_time: float = 9,
    ) -> None:
        """Handles the place screw request"""
        # Move to the target location

        if isinstance(target, LocationArgument):
            target_location = target.representation
        elif isinstance(target, list):
            target_location = target

        above_target = deepcopy(target_location)
        above_target[2] += 0.03
        self.ur.movel(above_target, self.acceleration, self.velocity)
        self.ur.movel(target_location, 0.2, 0.2)

        target_pose = [0, 0, 0.00021, 0, 0, 3.14]  # Setting the screw drive motion
        self.logger.info("Screwing down")

        self.ur.speedl_tool(
            target_pose, 2, screw_time
        )  # This will perform screw driving motion for defined number of seconds
        sleep(screw_time)
        self.logger.info("Screw drive motion completed")

        self.ur.translate_tool([0, 0, -0.03], 0.5, 0.5)

    def remove_cap(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
    ) -> None:
        """Handles the remove cap request"""
        self.open_gripper()
        if isinstance(source, LocationArgument):
            source_location = source.representation
        elif isinstance(source, list):
            source_location = source

        above_goal = deepcopy(source_location)
        above_goal[2] += 0.06
        self.ur.movel(above_goal, self.acceleration, self.velocity)
        self.ur.movel(source_location, 0.2, 0.2)

        self.close_gripper()

        if self.resource_client and isinstance(source, LocationArgument):  # Handle resources if configured
            popped_object, updated_resource = self.resource_client.pop(resource=source.resource_id)
            self.resource_client.push(resource=self.gripper_resource_id, child=popped_object)

        target_pose = [0, 0, -0.001, 0, 0, -3.14]  # Setting the screw drive motion
        self.logger.info("Removing cap")
        screw_time = 7
        self.ur.speedl_tool(
            target_pose, 2, screw_time
        )  # This will perform screw driving motion for defined number of seconds
        sleep(screw_time + 0.5)
        self.ur.translate_tool([0, 0, -0.03], 0.5, 0.5)

        self.home_robot(home)
        self.place(place_goal=target)
        self.home_robot(home)

    def place_cap(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
    ) -> None:
        """Handles the replace cap request"""

        self.pick(pick_goal=source)
        self.home_robot(home)

        if isinstance(target, LocationArgument):
            target_location = target.representation
        elif isinstance(target, list):
            target_location = target

        above_goal = deepcopy(target_location)
        above_goal[2] += 0.06
        self.ur.movel(above_goal, self.acceleration, self.velocity)
        self.ur.movel(target_location, 0.1, 0.1)

        # self.close_gripper()

        target_pose = [0, 0, 0.0001, 0, 0, 2.10]  # Setting the screw drive motion
        self.logger.info("Placing cap")
        screw_time = 6
        self.ur.speedl_tool(
            target_pose, 2, screw_time
        )  # This will perform screw driving motion for defined number of seconds
        sleep(screw_time)

        self.open_gripper()

        if self.resource_client and isinstance(target, LocationArgument):  # Handle resources if configured
            popped_object, updated_resource = self.resource_client.pop(resource=self.gripper_resource_id)
            self.resource_client.push(resource=target.resource_id, child=popped_object)

        self.ur.translate_tool([0, 0, -0.03], 0.5, 0.5)
        self.home_robot(home)

    def flip_object(
        self,
        target: Union[LocationArgument, list] = None,
        approach_axis: str = None,
    ) -> None:
        """Flips the object at the target location"""

        self.pick(pick_goal=target, approach_axis=approach_axis)

        cur_j = self.ur.getj()
        rotate_j = cur_j
        rotate_j[5] += radians(180)
        self.ur.movej(rotate_j, 0.6, 0.6)

        cur_l = self.ur.getl()
        target[3] = cur_l[3]
        target[4] = cur_l[4]
        target[5] = cur_l[5]

        self.place(place_goal=target, approach_axis=approach_axis)

    def transfer(
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
        try:
            self.logger.info("Starting transfer operation")
            self.pick(
                source=source,
                approach_axis=source_approach_axis,
                approach_distance=source_approach_distance,
            )
            self.logger.info("Pick completed")

            self.home_robot(home=home)

            self.place(
                target=target,
                approach_axis=target_approach_axis,
                approach_distance=target_approach_distance,
            )
            self.logger.info("Place completed")

        except Exception as e:
            self.logger.error(f"Error during transfer operation: {e}\n{traceback.format_exc()}")
            raise

    def screw_transfer(
        self,
        home: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        screwdriver_loc: Union[LocationArgument, list] = None,
        screw_loc: Union[LocationArgument, list] = None,
        screw_time: float = 9,
    ) -> None:
        """Handles the transfer request"""

        self.pick(
            pick_goal=screwdriver_loc,
        )  # Pick up the screwdriver bit
        self.home_robot(home=home)  # Move back to home position
        self.pick_screw(home=home, screw_loc=screw_loc)  # Pick up the screw
        self.place_screw(home=home, target=target, screw_time=screw_time)  # Drive the screwdriving motion
        self.home_robot(home=home)  # Move back to home position
        self.place(place_goal=screwdriver_loc)  # Place the screwdriver bit
        self.home_robot(home=home)


class VacuumGripperController:
    """Robotiq Vacuum Gripper Controller"""

    def __init__(
        self,
        IP: str = "146.137.240.38",
        PORT: int = 29999,
        gripper: bool = False,
    ):
        """Constructor for VacummGripperController"""
        super().__init__(IP=IP, PORT=PORT)
