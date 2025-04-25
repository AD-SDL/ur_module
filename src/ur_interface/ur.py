#!/usr/bin/env python3
"""Interface for UR Driver"""

import socket
from math import radians
from time import sleep
from typing import Union

import numpy as np
from madsci.client.resource_client import ResourceClient
from madsci.common.types.auth_types import OwnershipInfo
from madsci.common.types.location_types import LocationArgument
from urx import Robot

from ur_interface.ur_dashboard import UR_DASHBOARD
from ur_interface.ur_tools.gripper_controller import FingerGripperController
from ur_interface.ur_tools.ot_pipette_controller import OTPipetteController
from ur_interface.ur_tools.screwdriver_controller import ScrewdriverController
from ur_interface.ur_tools.tricontinent_pipette_controller import TricontinentPipetteController
from ur_interface.ur_tools.wm_tool_changer_controller import WMToolChangerController


class Connection:
    """Connection to the UR robot to be shared within UR driver"""

    def __init__(self, hostname: str = "146.137.240.38") -> None:
        """Connection class that creates a connection with the robot using URx Library"""
        self.hostname = hostname

        self.connection = None
        self.connect_ur()

    def connect_ur(self):
        """
        Description: Create conenction to the UR robot
        """

        for i in range(10):
            try:
                self.connection = Robot(self.hostname)

            except socket.error:
                print("Trying robot connection {}...".format(i))
                sleep(1)

            else:
                print("Successful ur connection")
                break

    def disconnect_ur(self):
        """
        Description: Disconnects the socket connection with the UR robot
        """
        self.connection.close()
        print("Robot connection is closed.")


class UR:
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
    ):
        """Constructor for the UR class.
        :param hostname: Hostname or ip.
        """

        if not hostname:
            raise TypeError("Hostname cannot be None Type!")

        self.hostname = hostname
        self.resource_client = resource_client
        self.tool_resource_id = tool_resource_id
        self.resource_owner = resource_owner

        self.acceleration = 0.5
        self.velocity = 0.5
        self.speed_ms = 0.750
        self.speed_rads = 0.750
        self.accel_mss = 1.200
        self.accel_radss = 1.200
        self.blend_radius_m = 0.001
        self.ref_frame = [0, 0, 0, 0, 0, 0]
        self.robot_current_joint_angles = None

        # if not self.hostname == "127.0.0.1":
        self.ur_dashboard = UR_DASHBOARD(hostname=self.hostname)
        self.ur = Connection(hostname=self.hostname)
        self.ur_connection = self.ur.connection

        self.gripper_speed = 255
        self.gripper_force = 255

        self.ur_connection.set_tcp((0, 0, 0, 0, 0, 0))
        self.get_movement_state()

        # TODO: get the information of what is the current tool attached to UR. Run a sanity check at the beginning to find out if a tool is connected

    def disconnect(self):
        "Disconnects the robot from URX and UR Dahsboard connections"
        self.ur.disconnect_ur()
        self.ur_dashboard.clear_operational_mode()
        self.ur_dashboard.disconnect()

    def get_movement_state(self) -> str:
        """Gets robot movement status by checking robot joint values.
        Return (str) READY if robot is not moving
                     BUSY if robot is moving
        """
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

    def home(self, home_location: Union[LocationArgument, list]) -> None:
        """Moves the robot to the home location.

        Args: home_location: 6 joint value location
        """

        print("Homing the robot...")
        if isinstance(home_location, LocationArgument):
            home_loc = home_location.location
        else:
            home_loc = home_location
        self.ur_connection.movej(home_loc, self.velocity, self.acceleration)
        print("Robot homed")

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
            print("Error in picking tool: ", err)

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
            print("Error in placing tool: ", err)

    def set_digital_io(self, channel: int = None, value: bool = None) -> None:
        """Sets digital I/O outputs to open an close the channel. This helps controlling the external tools

        Args
            channel (int): Channel number
            value (bool): False for close, True for open
        """
        if channel is None or value is None:
            print("Channel or value is not specified")
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
            raise Exception("Please provide both the source and target locations to make a transfer")

        self.home(home)

        try:
            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.tool_resource_id,
            )
            gripper_controller.connect_gripper()
            gripper_controller.velocity = self.velocity
            gripper_controller.acceleration = self.acceleration
            gripper_controller.gripper_speed = self.gripper_speed
            gripper_controller.gripper_force = self.gripper_force

            if gripper_open:
                gripper_controller.gripper_open = gripper_open
            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            gripper_controller.transfer(
                home=home,
                source=source,
                target=target,
                source_approach_axis=source_approach_axis,
                target_approach_axis=target_approach_axis,
                source_approach_distance=source_approach_distance,
                target_approach_distance=target_approach_distance,
            )
            print("Finished transfer")
            gripper_controller.disconnect_gripper()

        except Exception as err:
            print(err)

        finally:
            gripper_controller.disconnect_gripper()
            self.home(home)

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
            raise Exception("Please provide the source location to make a pick")

        self.home(home)

        try:
            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.tool_resource_id,
            )

            gripper_controller.connect_gripper()
            gripper_controller.velocity = self.velocity
            gripper_controller.acceleration = self.acceleration
            gripper_controller.gripper_speed = self.gripper_speed
            gripper_controller.gripper_force = self.gripper_force

            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            gripper_controller.pick(
                pick_goal=source,
                approach_axis=source_approach_axis,
                approach_distance=source_approach_distance,
            )
            print("Finished gripper pick")
            gripper_controller.disconnect_gripper()

        except Exception as err:
            print(err)

        finally:
            gripper_controller.disconnect_gripper()
            self.home(home)

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
            raise Exception("Please provide the target location to make a place")

        self.home(home)

        try:
            gripper_controller = FingerGripperController(
                hostname=self.hostname,
                ur=self.ur_connection,
                resource_client=self.resource_client,
                gripper_resource_id=self.tool_resource_id,
            )
            gripper_controller.connect_gripper()
            gripper_controller.velocity = self.velocity
            gripper_controller.acceleration = self.acceleration
            gripper_controller.gripper_speed = self.gripper_speed
            gripper_controller.gripper_force = self.gripper_force

            if gripper_open:
                gripper_controller.gripper_open = gripper_open

            gripper_controller.place(
                place_goal=target,
                approach_axis=target_approach_axis,
                approach_distance=target_approach_distance,
            )
            print("Finished gripper place")
            gripper_controller.disconnect_gripper()

        except Exception as err:
            print(err)

        finally:
            gripper_controller.disconnect_gripper()
            self.home(home)

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
            print(err)

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
            print(err)

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
            print(err)

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

            gripper_controller.pick(pick_goal=target, approach_axis=approach_axis)

            cur_j = self.ur_connection.getj()
            rotate_j = cur_j
            rotate_j[5] += radians(180)
            self.ur_connection.movej(rotate_j, 0.6, 0.6)

            cur_l = self.ur_connection.getl()
            target[3] = cur_l[3]
            target[4] = cur_l[4]
            target[5] = cur_l[5]

            gripper_controller.place(place_goal=target, approach_axis=approach_axis)
            self.home(home)

        except Exception as er:
            print(er)
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
            print(err)

        self.home(home)

    def pipette_transfer(
        self,
        home: Union[LocationArgument, list] = None,
        tip_loc: Union[LocationArgument, list] = None,
        tip_trash: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        volume: int = 10,
    ) -> None:
        """
        Make a liquid transfer using the pipette. This function uses linear motions to perform the pick and place movements.

        Args
            home (Union[LocationArgument, list]): Home location
            tip_loc (Union[LocationArgument, list]): Pipette tip location
            tip_trash (Union[LocationArgument, list]): Tip trash location
            source (str): Source location
            target (str): Target location
            volume (int): Pipette transfer volume. Unit number of steps. Each step is 1 mL
        """
        if not tip_loc or not source:
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
            pipette.pick_tip(tip_loc=tip_loc)
            self.home(home)
            pipette.transfer_sample(home=home, sample_aspirate=source, sample_dispense=target, vol=volume)
            pipette.eject_tip(eject_tip_loc=tip_trash, approach_axis="y")
            pipette.disconnect_pipette()
            print("Disconnecting from the pipette")
        except Exception as err:
            print(err)

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

        print("Running the URP program: ", program_name)
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
