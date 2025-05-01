"""REST-based node for UR robots"""

from typing import Optional

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
from pydantic.networks import AnyUrl
from typing_extensions import Annotated

from ur_interface.ur import UR
from ur_interface.ur_kinematics import get_pose_from_joint_angles


class URNodeConfig(RestNodeConfig):
    """Configuration for the UR node module."""

    ur_ip: str
    resource_manager_url: Optional[AnyUrl] = None


class URNode(RestNode):
    """A Rest Node object to control UR robots"""

    ur_interface: UR = None
    config_model = URNodeConfig

    def startup_handler(self) -> None:
        """Called to (re)initialize the node. Should be used to open connections to devices or initialize any other resources."""

        try:
            if self.config.resource_manager_url:
                self.resource_client = ResourceClient(self.config.resource_manager_url)
                self.resource_owner = OwnershipInfo(node_id=self.node_definition.node_id)

            else:
                self.resource_client = None

            self.logger.log("Node initializing...")
            self.ur_interface = UR(
                hostname=self.config.ur_ip,
                resource_client=self.resource_client,
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
        velocity: Optional[float] = None,
        acceleration: Optional[float] = None,
        gripper_speed: Optional[float] = None,
        gripper_force: Optional[float] = None,
    ):
        """Configure the robot's movement parameters for subsequent transfers"""
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
        joints: Annotated[LocationArgument, "Joint angles to move to"],
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
        target: Annotated[LocationArgument, "Linear location to move to"],
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

    @action(
        name="gripper_transfer",
        description="Execute a transfer in between source and target locations using Robotiq grippers",
    )
    def gripper_transfer(
        self,
        home: Annotated[LocationArgument, "Home location"],
        source: Annotated[LocationArgument, "Location to transfer sample from"],
        target: Annotated[LocationArgument, "Location to transfer sample to"],
        source_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        target_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        source_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        target_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
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

            source.location = get_pose_from_joint_angles(source.location)
            target.location = get_pose_from_joint_angles(target.location)

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
        home: Annotated[LocationArgument, "Home location"],
        source: Annotated[LocationArgument, "Location to transfer sample from"],
        source_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        source_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = "0.05",
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = "255",
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

            source.location = get_pose_from_joint_angles(source.location)

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
        home: Annotated[LocationArgument, "Home location"],
        target: Annotated[LocationArgument, "Location to transfer sample to"],
        target_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        target_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = "0.05",
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
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

            target.location = get_pose_from_joint_angles(target.location)

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
        home: Annotated[LocationArgument, "Home location"],
        tool_loc: Annotated[LocationArgument, "Tool location"],
        docking_axis: Annotated[Optional[str], "Docking axis, (X/Y/Z)"] = "y",
        payload: Annotated[Optional[float], "Tool payload"] = None,
        tool_name: Annotated[Optional[str], "Tool name)"] = None,
    ):
        """Pick a tool with the UR"""

        if not tool_loc or not home:  # Return Fail
            return ActionFailed(errors="tool_loc and home locations must be provided")

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
        home: Annotated[LocationArgument, "Home location"],
        tool_docking: Annotated[LocationArgument, "Tool docking location"],
        docking_axis: Annotated[Optional[str], "Docking axis, (X/Y/Z)"] = "y",
        tool_name: Annotated[Optional[str], "Tool name)"] = None,
    ):
        """Place a tool with the UR"""
        try:
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
        home: Annotated[LocationArgument, "Home location"],
        screwdriver_loc: Annotated[LocationArgument, "Screwdriver location"],
        screw_loc: Annotated[LocationArgument, "Screw location"],
        target: Annotated[LocationArgument, "Location where the srewdriving will be performed"],
        screw_time: Annotated[Optional[int], "Srew time in seconds"] = 9,
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
    ):
        """Make a screwdriving transfer using Robotiq gripper and custom screwdriving bits with UR"""

        if not home or not screwdriver_loc or not screw_loc or not target:
            return ActionFailed(errors="screwdriver_loc, screw_loc and home locations must be provided")
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
        home: Annotated[LocationArgument, "Home location"],
        source: Annotated[LocationArgument, "Initial location of the sample"],
        target: Annotated[LocationArgument, "Target location of the sample"],
        tip_loc: Annotated[LocationArgument, "New tip location"],
        tip_trash: Annotated[LocationArgument, "Tip trash location"],
        volume: Annotated[float, "Set a volume in micro liters"],
    ):
        """Make a pipette transfer for the defined volume with UR"""
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
        name="pick_and_flip_object",
        description="Picks and flips an object 180 degrees",
    )
    def pick_and_flip_object(
        self,
        home: Annotated[LocationArgument, "Home location"],
        target: Annotated[LocationArgument, "Location of the object"],
        approach_axis: Annotated[Optional[str], "Approach axis, (X/Y/Z)"] = "z",
        target_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
    ):
        """Picks and flips an object 180 degrees with UR"""
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
        home: Annotated[LocationArgument, "Home location"],
        source: Annotated[LocationArgument, "Location of the vial cap"],
        target: Annotated[
            LocationArgument,
            "Location of where the cap will be placed after it is removed from the vail",
        ],
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
    ):
        """Remove caps from sample vials with UR"""
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
        home: Annotated[LocationArgument, "Home location"],
        source: Annotated[LocationArgument, "Vail cap initial location"],
        target: Annotated[LocationArgument, "The vail location where the cap will installed"],
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
    ):
        """Places caps back to sample vials with UR"""
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
            return AdminCommandResponse(data={"Joint Angles": self.ur_interface.ur_connection.getj()})
        except Exception:
            return AdminCommandResponse(success=False)


if __name__ == "__main__":
    ur_node = URNode()
    ur_node.start_node()
