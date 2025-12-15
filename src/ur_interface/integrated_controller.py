from copy import deepcopy
from enum import Enum
import logging
import traceback
from typing import Optional, Union

import math3d as m3
import numpy as np
from madsci.client.resource_client import ResourceClient
from madsci.common.types.auth_types import OwnershipInfo
from madsci.common.types.location_types import LocationArgument

from ur_interface.ur_controller import URController
from ur_interface.ur_dashboard import URDashboard

from ur_interface.ur_tools.wm_tool_changer_controller import WMToolChangerController
from ur_interface.ur_tools.grippers.robotiq_gripper_interface import RobotiqGripper


class UREndEffector(str, Enum): 
    Robotiq2FingerGripper = "ROBOTIQ2FINGERGRIPPER"
    SCREWDRIVER = "SCREWDRIVER"
    PIPETTE = "PIPETTE"
end_effectors = {
    UREndEffector.Robotiq2FingerGripper: RobotiqGripper
}
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
        end_effector: Optional[UREndEffector] = None,
        logger: Optional[logging.Logger] = None
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
            self.ur_dashboard = URDashboard(hostname=self.hostname)
            self.ur_controller = URController(
                hostname=self.hostname, logger=self.logger, tcp_pose=tcp_pose, base_reference_frame=base_reference_frame
            )

            self.ur_connection.ur_connection.set_tcp(tcp_pose)
            self.create_end_effector_controller()
        except Exception as e:
            self.logger.error(f"Failed to initialize UR: {e}\n{traceback.format_exc()}")
            raise e
    def create_end_effector_controller(self) -> None:
        """Create appropriate end-effector controller."""
        self.end_effector = end_effectors[self.end_effector](hostname=self.hostname)
        self.logger.log_info("Creating Robotiq 2-finger gripper controller")
        if self.end_effector.tool_params:
            self.ur_controller.ur_connection.set_tool_communication(
                                **self.end_effector.tool_params
        )
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
                raise ValueError("Please provide either a LocationArgument or a list for the target location")

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
            self.ur_controller.move_to_location(above_goal, linear_motion=True)
            self.logger.debug("Moving to goal position")
            self.ur_controller.move_to_location(above_goal, linear_motion=True)

            self.gripper_controller.open_gripper()

            if self.resource_client and isinstance(target, LocationArgument):  # Handle resources if configured
                try:
                    popped_object, updated_resource = self.resource_client.pop(resource=self.gripper_resource_id)
                    self.resource_client.push(resource=target.resource_id, child=popped_object)
                except Exception as e:
                    self.logger.error(f"Resource client error during place: {e}\n{traceback.format_exc()}")

            self.logger.debug("Moving back to above goal position")
            self.ur.movel(above_goal, self.acceleration, self.velocity)
            self.logger.info("Place operation completed successfully")

        except Exception as e:
            self.logger.error(f"Error during place operation: {e}\n{traceback.format_exc()}")
            raise e

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
            time.sleep(2)

        self.ur_dashboard.load_program(program_path=ur_program_path)
        time.sleep(2)
        self.ur_dashboard.run_program()
        time.sleep(5)

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