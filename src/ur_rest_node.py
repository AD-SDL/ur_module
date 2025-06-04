"""REST-based node for UR robots"""

from typing import Optional, Union

from madsci.client.resource_client import ResourceClient
from madsci.common.types.action_types import ActionFailed, ActionSucceeded
from madsci.common.types.admin_command_types import AdminCommandResponse
from madsci.common.types.auth_types import OwnershipInfo
from madsci.common.types.location_types import LocationArgument
from madsci.common.types.node_types import RestNodeConfig
from madsci.common.types.resource_types.definitions import (
    PoolResourceDefinition,
    SlotResourceDefinition,
)
from madsci.node_module.helpers import action
from madsci.node_module.rest_node_module import RestNode
from typing_extensions import Annotated

from ur_interface.ur import UR
from ur_interface.ur_kinematics import get_pose_from_joint_angles
from ur_interface.ur_tools.gripper_controller import FingerGripperController


class URNodeConfig(RestNodeConfig):
    """Configuration for the UR node module."""

    ur_ip: str
    tcp_pose: list = [0, 0, 0, 0, 0, 0]
    ur_model: str = "UR5e"


class URNode(RestNode):
    """A Rest Node object to control UR robots"""

    ur_interface: UR = None
    config_model = URNodeConfig

    def startup_handler(self) -> None:
        """Called to (re)initialize the node. Should be used to open connections to devices or initialize any other resources."""
        try:
            if self.config.resource_server_url:
                self.resource_client = ResourceClient(self.config.resource_server_url)
                self.resource_owner = OwnershipInfo(node_id=self.node_definition.node_id)

            else:
                self.resource_client = None

            self.logger.log("Node initializing...")
            self.ur_interface = UR(
                hostname=self.config.ur_ip,
                resource_client=self.resource_client,
                tcp_pose=self.config.tcp_pose,
            )
            self.tool_resource = None

        except Exception as err:
            self.logger.log_error(f"Error starting the UR Node: {err}")
            self.startup_has_run = False
        else:
            self.startup_has_run = True
            self.logger.log("UR node initialized!")

    def shutdown_handler(self) -> None:
        """Called to shutdown the node. Should be used to close connections to devices or release any other resources."""
        try:
            self.logger.log("Shutting down")
            self.ur_interface.disconnect()
            self.shutdown_has_run = True
            del self.ur_interface
            self.ur_interface = None
            self.logger.log("Shutdown complete.")
        except Exception as err:
            self.logger.log_error(f"Error shutting down the UR Node: {err}")

    def state_handler(self) -> None:
        """Periodically called to update the current state of the node."""
        if self.ur_interface:
            # Getting robot state
            self.ur_interface.ur_dashboard.get_overall_robot_status()
            movement_state, current_location = self.ur_interface.get_movement_state()
        else:
            self.logger.log_error("UR interface is not initialized")
            return
        if "NORMAL" not in self.ur_interface.ur_dashboard.safety_status:
            self.node_state = {
                "ur_status_code": "ERROR",
                "current_joint_angles": current_location,
            }
            self.logger.log_error(f"UR ERROR: {self.ur_interface.ur_dashboard.safety_status}")

        elif movement_state == "BUSY":
            self.node_state = {
                "ur_status_code": "BUSY",
                "current_joint_angles": current_location,
            }
            self.logger.info("BUSY")

        elif movement_state == "READY":
            self.node_state = {
                "ur_status_code": "READY",
                "current_joint_angles": current_location,
            }
        else:
            self.node_state = {
                "ur_status_code": "UNKOWN",
                "current_joint_angles": current_location,
            }

    @action(name="getj", description="Get joint angles")
    def getj(self):
        """Get joint positions"""
        joints = self.ur_interface.ur_connection.getj()
        self.logger.log_info(joints)
        return ActionSucceeded(data={"joints": joints})

    @action(name="getl", description="Get linear positions")
    def getl(self):
        """Get linear position"""
        lin_pos = self.ur_interface.ur_connection.getl()
        self.logger.log_info(lin_pos)
        return ActionSucceeded(data={"lin_pos": lin_pos})

    @action(name="set_freedrive", description="Free robot joints")
    def set_freedrive(self, timeout: Annotated[int, "how long to do freedrive"] = 60):
        """set the robot into freedrive"""
        self.ur_interface.ur_connection.set_freedrive(True, timeout)
        return ActionSucceeded()

    @action(name="set_movement_params", description="Set speed and acceleration parameters")
    def set_movement_params(
        self,
        tcp_pose: Optional[list] = None,
        velocity: Optional[float] = None,
        acceleration: Optional[float] = None,
        gripper_speed: Optional[float] = None,
        gripper_force: Optional[float] = None,
    ):
        """Configure the robot's movement parameters for subsequent transfers"""
        if tcp_pose is not None:
            self.ur_interface.ur_connection.set_tcp(tcp_pose)
        if velocity is not None:
            self.ur_interface.velocity = velocity
        if acceleration is not None:
            self.ur_interface.acceleration = acceleration
        if gripper_speed is not None:
            self.ur_interface.gripper_speed = gripper_speed
        if gripper_force is not None:
            self.ur_interface.gripper_force = gripper_force
        return ActionSucceeded()

    @action(name="movej", description="Move the robot using joint angles")
    def movej(
        self,
        joints: Annotated[Union[LocationArgument, list], "Joint angles to move to"],
        acceleration: Annotated[Optional[float], "Acceleration"] = 0.6,
        velocity: Annotated[Optional[float], "Velocity"] = 0.6,
    ):
        """Move the robot using joint angles"""
        try:
            self.logger.log(f"Move joints: {joints.location}")
            self.ur_interface.ur_connection.movej(joints=joints.location, acc=acceleration, vel=velocity)

        except Exception as err:
            self.logger.log_error(err)

        return ActionSucceeded()

    @action(name="movel", description="Move the robot using linar motion")
    def movel(
        self,
        target: Annotated[Union[LocationArgument, list], "Linear location to move to"],
        acceleration: Annotated[Optional[float], "Acceleration"] = 0.6,
        velocity: Annotated[Optional[float], "Velocity"] = 0.6,
    ):
        """Move the robot using linear motion"""
        try:
            self.logger.log(f"Move location: {target.location}")
            self.ur_interface.ur_connection.movel(tpose=target.location, acc=acceleration, vel=velocity)

        except Exception as err:
            self.logger.log_error(err)

        return ActionSucceeded()

    @action(name="toggle_gripper", description="Move the robot gripper")
    def toggle_gripper(
        self,
        open: Annotated[bool, "Open?"] = False,
        close: Annotated[bool, "Close?"] = False,
    ):
        """Open or close the robot gripper."""
        try:
            gripper = FingerGripperController(hostname=self.config.ur_ip, ur=self.ur_interface)
            self.logger.log("Connecting to gripper...")
            gripper.connect_gripper()
            self.logger.log("Gripper connected")
            if open:
                gripper.open_gripper()
                self.logger.log("Gripper opened")
            elif close:
                gripper.close_gripper()
                self.logger.log("Gripper closed")
            else:
                self.logger.log("No action taken")

        except Exception as err:
            self.logger.log_error(err)
        else:
            gripper.disconnect_gripper()
            self.logger.log("Gripper disconnected")
        return ActionSucceeded()

    @action(
        name="gripper_transfer",
        description="Execute a transfer in between source and target locations using Robotiq grippers",
    )
    def gripper_transfer(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        source: Annotated[Union[LocationArgument, list], "Location to transfer sample from"],
        target: Annotated[Union[LocationArgument, list], "Location to transfer sample to"],
        source_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        target_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        source_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        target_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Make a transfer using the finger gripper. This function uses linear motions to perform the pick and place movements."""
        try:
            # Check if the source, target and home locations are provided
            if not source or not target or not home:  # Return Fail
                return ActionFailed(errors="Source, target and home locations must be provided")

            if self.resource_client:
                # If the gripper resource is not initialized, initialize it
                self.tool_resource = self.resource_client.init_resource(
                    SlotResourceDefinition(
                        resource_name="ur_gripper",
                        owner=self.resource_owner,
                    )
                )
                self.ur_interface.tool_resource_id = self.tool_resource.resource_id

            if joint_angle_locations and isinstance(source, LocationArgument):
                source.location = get_pose_from_joint_angles(joints=source.location, robot_model=self.config.ur_model)
                target.location = get_pose_from_joint_angles(joints=target.location, robot_model=self.config.ur_model)
            elif joint_angle_locations and isinstance(source, list):
                source = get_pose_from_joint_angles(joints=source, robot_model=self.config.ur_model)
                target = get_pose_from_joint_angles(joints=target, robot_model=self.config.ur_model)

            self.ur_interface.gripper_transfer(
                home=home,
                source=source,
                target=target,
                source_approach_distance=source_approach_distance,
                target_approach_distance=target_approach_distance,
                source_approach_axis=source_approach_axis,
                target_approach_axis=target_approach_axis,
                gripper_open=gripper_open,
                gripper_close=gripper_close,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action()
    def gripper_pick(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        source: Annotated[Union[LocationArgument, list], "Location to transfer sample from"],
        source_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        source_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Use the gripper to pick a piece of labware from the specified source"""
        try:
            if self.resource_client:
                # If the gripper resource is not initialized, initialize it
                self.tool_resource = self.resource_client.init_resource(
                    SlotResourceDefinition(
                        resource_name="ur_gripper",
                        owner=self.resource_owner,
                    )
                )
                self.ur_interface.tool_resource_id = self.tool_resource.resource_id

            if joint_angle_locations and isinstance(source, LocationArgument):
                source.location = get_pose_from_joint_angles(joints=source.location, robot_model=self.config.ur_model)
            elif joint_angle_locations and isinstance(source, list):
                source = get_pose_from_joint_angles(joints=source, robot_model=self.config.ur_model)

            self.ur_interface.gripper_pick(
                home=home,
                source=source,
                source_approach_distance=source_approach_distance,
                source_approach_axis=source_approach_axis,
                gripper_close=gripper_close,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action()
    def gripper_place(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        target: Annotated[Union[LocationArgument, list], "Location to transfer sample to"],
        target_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        target_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Use the gripper to place a piece of labware at the target."""
        try:
            if self.resource_client:
                # If the gripper resource is not initialized, initialize it
                self.tool_resource = self.resource_client.init_resource(
                    SlotResourceDefinition(
                        resource_name="ur_gripper",
                        owner=self.resource_owner,
                    )
                )
                self.ur_interface.tool_resource_id = self.tool_resource.resource_id

            if joint_angle_locations and isinstance(target, LocationArgument):
                target.location = get_pose_from_joint_angles(joints=target.location, robot_model=self.config.ur_model)
            elif joint_angle_locations and isinstance(target, list):
                target = get_pose_from_joint_angles(joints=target, robot_model=self.config.ur_model)

            self.ur_interface.gripper_place(
                home=home,
                target=target,
                target_approach_distance=target_approach_distance,
                target_approach_axis=target_approach_axis,
                gripper_open=gripper_open,
            )
        except Exception as err:
            return ActionFailed(errors=err)
        return ActionSucceeded()

    @action(
        name="pick_tool",
        description="Picks up a tool using the provided tool location",
    )
    def pick_tool(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        tool_loc: Annotated[Union[LocationArgument, list], "Tool location"],
        docking_axis: Annotated[Optional[str], "Docking axis, (X/Y/Z)"] = "y",
        payload: Annotated[Optional[float], "Tool payload"] = None,
        tool_name: Annotated[Optional[str], "Tool name)"] = None,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Pick a tool with the UR"""

        if not tool_loc or not home:  # Return Fail
            return ActionFailed(errors="tool_loc and home locations must be provided")

        if joint_angle_locations and isinstance(tool_loc, LocationArgument):
            tool_loc.location = get_pose_from_joint_angles(joints=tool_loc.location, robot_model=self.config.ur_model)
        elif joint_angle_locations and isinstance(tool_loc, list):
            tool_loc = get_pose_from_joint_angles(joints=tool_loc, robot_model=self.config.ur_model)

        try:
            self.ur_interface.pick_tool(
                home=home,
                tool_loc=tool_loc,
                docking_axis=docking_axis,
                payload=payload,
                tool_name=tool_name,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(name="Place_tool", description="Places the attached tool back to the provided tool docking location")
    def place_tool(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        tool_docking: Annotated[Union[LocationArgument, list], "Tool docking location"],
        docking_axis: Annotated[Optional[str], "Docking axis, (X/Y/Z)"] = "y",
        tool_name: Annotated[Optional[str], "Tool name)"] = None,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Place a tool with the UR"""
        try:
            if joint_angle_locations and isinstance(tool_docking, LocationArgument):
                tool_docking.location = get_pose_from_joint_angles(
                    joints=tool_docking.location, robot_model=self.config.ur_model
                )
            elif joint_angle_locations and isinstance(tool_docking, list):
                tool_docking = get_pose_from_joint_angles(joints=tool_docking, robot_model=self.config.ur_model)

            self.ur_interface.place_tool(
                home=home,
                tool_loc=tool_docking,
                docking_axis=docking_axis,
                tool_name=tool_name,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(
        name="gripper_screw_transfer",
        description="Performs a screw transfer using the Robotiq gripper and custom screwdriving bits",
    )
    def gripper_screw_transfer(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        screwdriver_loc: Annotated[Union[LocationArgument, list], "Screwdriver location"],
        screw_loc: Annotated[Union[LocationArgument, list], "Screw location"],
        target: Annotated[Union[LocationArgument, list], "Location where the srewdriving will be performed"],
        screw_time: Annotated[Optional[int], "Srew time in seconds"] = 9,
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Make a screwdriving transfer using Robotiq gripper and custom screwdriving bits with UR"""

        if not home or not screwdriver_loc or not screw_loc or not target:
            return ActionFailed(errors="screwdriver_loc, screw_loc and home locations must be provided")

        if joint_angle_locations and isinstance(screwdriver_loc, LocationArgument):
            screwdriver_loc.location = get_pose_from_joint_angles(
                joints=screwdriver_loc.location, robot_model=self.config.ur_model
            )
            screw_loc.location = get_pose_from_joint_angles(joints=screw_loc.location, robot_model=self.config.ur_model)
            target.location = get_pose_from_joint_angles(joints=target.location, robot_model=self.config.ur_model)
        elif joint_angle_locations and isinstance(screwdriver_loc, list):
            screwdriver_loc = get_pose_from_joint_angles(joints=screwdriver_loc, robot_model=self.config.ur_model)
            screw_loc = get_pose_from_joint_angles(joints=screw_loc, robot_model=self.config.ur_model)
            target = get_pose_from_joint_angles(joints=target, robot_model=self.config.ur_model)

        try:
            self.ur_interface.gripper_screw_transfer(
                home=home,
                screwdriver_loc=screwdriver_loc,
                screw_loc=screw_loc,
                screw_time=screw_time,
                target=target,
                gripper_open=gripper_open,
                gripper_close=gripper_close,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(
        name="pipette_transfer",
        description="Make a pipette transfer to transfer sample liquids in between two locations",
    )
    def pipette_transfer(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        source: Annotated[Union[LocationArgument, list], "Initial location of the sample"],
        target: Annotated[Union[LocationArgument, list], "Target location of the sample"],
        tip_loc: Annotated[Union[LocationArgument, list], "New tip location"],
        tip_trash: Annotated[Union[LocationArgument, list], "Tip trash location"],
        volume: Annotated[float, "Set a volume in micro liters"],
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Make a pipette transfer for the defined volume with UR"""
        if not home or not source or not target or not tip_loc or not tip_trash:
            return ActionFailed(errors="home, source, target, tip_loc and tip_trash locations must be provided")

        if joint_angle_locations and isinstance(source, LocationArgument):
            source.location = get_pose_from_joint_angles(joints=source.location, robot_model=self.config.ur_model)
            target.location = get_pose_from_joint_angles(joints=target.location, robot_model=self.config.ur_model)
            tip_loc.location = get_pose_from_joint_angles(joints=tip_loc.location, robot_model=self.config.ur_model)
            tip_trash.location = get_pose_from_joint_angles(joints=tip_trash.location, robot_model=self.config.ur_model)
        elif joint_angle_locations and isinstance(source, list):
            source = get_pose_from_joint_angles(joints=source, robot_model=self.config.ur_model)
            target = get_pose_from_joint_angles(joints=target, robot_model=self.config.ur_model)
            tip_loc = get_pose_from_joint_angles(joints=tip_loc, robot_model=self.config.ur_model)
            tip_trash = get_pose_from_joint_angles(joints=tip_trash, robot_model=self.config.ur_model)

        try:
            if self.resource_client:
                # If the pipette resource is not initialized, initialize it
                self.tool_resource = self.resource_client.init_resource(
                    PoolResourceDefinition(
                        resource_name="ur_pipette",
                        owner=self.resource_owner,
                    )
                )
                self.ur_interface.tool_resource_id = self.tool_resource.resource_id

            self.ur_interface.pipette_transfer(
                home=home,
                tip_loc=tip_loc,
                tip_trash=tip_trash,
                source=source,
                target=target,
                volume=volume,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(
        name="pipette_pick_and_move_sample",
        description="Picks and moves a sample using the pipette",
    )
    def pipette_pick_and_move_sample(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location in joint angles"],
        sample_loc: Annotated[Union[LocationArgument, list], "Sample location"],
        target: Annotated[Union[LocationArgument, list], "Location of the object"],
        volume: Annotated[int, "Set a volume in micro liters"] = 10,
        safe_waypoint: Annotated[Union[LocationArgument, list], "Safe waypoint in joint angles"] = None,
        tip_loc: Annotated[Union[LocationArgument, list], "Tip location"] = None,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Picks and moves a sample with UR"""

        if joint_angle_locations and isinstance(target, LocationArgument):
            target.location = get_pose_from_joint_angles(joints=target.location, robot_model=self.config.ur_model)
            sample_loc.location = get_pose_from_joint_angles(
                joints=sample_loc.location, robot_model=self.config.ur_model
            )
            tip_loc.location = (
                get_pose_from_joint_angles(joints=tip_loc.location, robot_model=self.config.ur_model)
                if tip_loc
                else None
            )
        elif joint_angle_locations and isinstance(target, list):
            target = get_pose_from_joint_angles(joints=target, robot_model=self.config.ur_model)
            sample_loc = get_pose_from_joint_angles(joints=sample_loc, robot_model=self.config.ur_model)
            tip_loc = get_pose_from_joint_angles(joints=tip_loc, robot_model=self.config.ur_model) if tip_loc else None

        try:
            self.ur_interface.pipette_pick_and_move_sample(
                home=home,
                safe_waypoint=safe_waypoint,
                sample_loc=sample_loc,
                target=target,
                volume=volume,
                tip_loc=tip_loc,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(
        name="pipette_dispense_and_retrieve",
        description="Dispenses a sample and retrieves the pipette tip",
    )
    def pipette_dispense_and_retrieve(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location in joint angles"],
        target: Annotated[Union[LocationArgument, list], "Location of the object"],
        volume: Annotated[int, "Set a volume in micro liters"] = 10,
        safe_waypoint: Annotated[Union[LocationArgument, list], "Safe waypoint in joint angles"] = None,
        tip_trash: Annotated[Union[LocationArgument, list], "Tip trash location"] = None,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Dispenses a sample and retrieves the pipette with UR"""

        if joint_angle_locations and isinstance(target, LocationArgument):
            target.location = get_pose_from_joint_angles(joints=target.location, robot_model=self.config.ur_model)
            tip_trash.location = (
                get_pose_from_joint_angles(joints=tip_trash.location, robot_model=self.config.ur_model)
                if tip_trash
                else None
            )
        elif joint_angle_locations and isinstance(target, list):
            target = get_pose_from_joint_angles(joints=target, robot_model=self.config.ur_model)
            tip_trash = (
                get_pose_from_joint_angles(joints=tip_trash, robot_model=self.config.ur_model) if tip_trash else None
            )

        try:
            self.ur_interface.pipette_dispense_and_retrieve(
                home=home,
                safe_waypoint=safe_waypoint,
                target=target,
                volume=volume,
                tip_trash=tip_trash,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(
        name="pick_and_flip_object",
        description="Picks and flips an object 180 degrees",
    )
    def pick_and_flip_object(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        target: Annotated[Union[LocationArgument, list], "Location of the object"],
        approach_axis: Annotated[Optional[str], "Approach axis, (X/Y/Z)"] = "z",
        target_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Picks and flips an object 180 degrees with UR"""

        if not home or not target:
            return ActionFailed(errors="home and target locations must be provided")
        if joint_angle_locations and isinstance(target, LocationArgument):
            target.location = get_pose_from_joint_angles(joints=target.location, robot_model=self.config.ur_model)
        elif joint_angle_locations and isinstance(target, list):
            target = get_pose_from_joint_angles(joints=target, robot_model=self.config.ur_model)

        try:
            self.ur_interface.pick_and_flip_object(
                home=home,
                target=target,
                approach_axis=approach_axis,
                target_approach_distance=target_approach_distance,
                gripper_open=gripper_open,
                gripper_close=gripper_close,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(
        name="remove_cap",
        description="Removes caps from sample vials",
    )
    def remove_cap(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        source: Annotated[Union[LocationArgument, list], "Location of the vial cap"],
        target: Annotated[
            Union[LocationArgument, list],
            "Location of where the cap will be placed after it is removed from the vail",
        ],
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Remove caps from sample vials with UR"""
        if not home or not source or not target:
            return ActionFailed(errors="home, source and target locations must be provided")
        if joint_angle_locations and isinstance(source, LocationArgument):
            source.location = get_pose_from_joint_angles(joints=source.location, robot_model=self.config.ur_model)
            target.location = get_pose_from_joint_angles(joints=target.location, robot_model=self.config.ur_model)
        elif joint_angle_locations and isinstance(source, list):
            source = get_pose_from_joint_angles(joints=source, robot_model=self.config.ur_model)
            target = get_pose_from_joint_angles(joints=target, robot_model=self.config.ur_model)

        try:
            self.ur_interface.remove_cap(
                home=home,
                source=source,
                target=target,
                gripper_open=gripper_open,
                gripper_close=gripper_close,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(
        name="place_cap",
        description="Places caps back to sample vials",
    )
    def place_cap(
        self,
        home: Annotated[Union[LocationArgument, list], "Home location"],
        source: Annotated[Union[LocationArgument, list], "Vail cap initial location"],
        target: Annotated[Union[LocationArgument, list], "The vail location where the cap will installed"],
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
        joint_angle_locations: Annotated[bool, "Use joint angles for all the locations"] = True,
    ):
        """Places caps back to sample vials with UR"""
        if not home or not source or not target:
            return ActionFailed(errors="home, source and target locations must be provided")
        if joint_angle_locations and isinstance(source, LocationArgument):
            source.location = get_pose_from_joint_angles(joints=source.location, robot_model=self.config.ur_model)
            target.location = get_pose_from_joint_angles(joints=target.location, robot_model=self.config.ur_model)
        elif joint_angle_locations and isinstance(source, list):
            source = get_pose_from_joint_angles(joints=source, robot_model=self.config.ur_model)
            target = get_pose_from_joint_angles(joints=target, robot_model=self.config.ur_model)

        try:
            self.ur_interface.place_cap(
                home=home,
                source=source,
                target=target,
                gripper_open=gripper_open,
                gripper_close=gripper_close,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(
        name="run_urp_program",
        description="Runs a URP program on the UR",
    )
    def run_urp_program(
        self,
        transfer_file_path=Annotated[str, "Transfer file path"],
        program_name=Annotated[str, "Program name"],
    ):
        """Run an URP program on the UR"""
        try:
            self.ur_interface.run_urp_program(
                transfer_file_path=transfer_file_path,
                program_name=program_name,
            )
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    @action(
        name="set_digital_io",
        description="Sets a channel IO output on the UR",
    )
    def set_digital_io(
        self,
        channel=Annotated[int, "Channel number"],
        value=Annotated[bool, "True/False"],
    ):
        """Sets a channel IO output on the UR"""
        try:
            self.ur_interface.set_digital_io(channel=channel, value=value)
        except Exception as err:
            return ActionFailed(errors=err)

        return ActionSucceeded()

    def get_location(self) -> AdminCommandResponse:
        """Return the current position of the ur robot"""
        try:
            return AdminCommandResponse(data=self.ur_interface.ur_connection.getj())
        except Exception:
            return AdminCommandResponse(success=False)


if __name__ == "__main__":
    ur_node = URNode()
    ur_node.start_node()
