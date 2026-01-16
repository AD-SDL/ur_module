"""REST-based node for UR robots"""

from typing import Optional, Union

from madsci.client.resource_client import ResourceClient
from madsci.common.types.admin_command_types import AdminCommandResponse
from madsci.common.types.location_types import LocationArgument
from madsci.common.types.node_types import RestNodeConfig
from madsci.common.types.resource_types import Pool, Slot
from madsci.node_module.helpers import action
from madsci.node_module.rest_node_module import RestNode
from pydantic import AnyUrl
from typing_extensions import Annotated

from ur_interface.integrated_controller import IntegratedController, UREndEffector


class URNodeConfig(RestNodeConfig):
    """Configuration for the UR node module."""

    ur_ip: Optional[str] = None
    tcp_pose: list = [0, 0, 0, 0, 0, 0]
    base_reference_frame: Optional[list] = None
    ur_model: str = "UR5e"
    end_effector: Optional[UREndEffector] = None
    resource_manager_url: Optional[AnyUrl] = None


class URNode(RestNode):
    """A Rest Node object to control UR robots"""

    integrated_controller: IntegratedController = None
    config: URNodeConfig = URNodeConfig()
    config_model = URNodeConfig

    def startup_handler(self) -> None:
        """Called to (re)initialize the node. Should be used to open connections to devices or initialize any other resources."""
        self.resource_client = None
        self.end_effector_resource_id = None
        if self.config.resource_manager_url is not None:
            self.resource_client = ResourceClient(self.config.resource_manager_url)
            self._create_ur_resources()
        self.logger.log("Node initializing...")
        self.integrated_controller = IntegratedController(
            hostname=self.config.ur_ip,
            resource_client=self.resource_client if self.config.use_resources else None,
            end_effector_resource_id=self.end_effector_resource_id,
            tcp_pose=self.config.tcp_pose,
            base_reference_frame=self.config.base_reference_frame,
            end_effector=self.config.end_effector,
            logger=self.logger,
        )
        self.tool_resource = None
        self.current_location = None
        self.startup_has_run = True
        self.logger.log("UR node initialized!")

    def _create_ur_resources(self) -> None:
        """Create all UR-specific resource templates."""
        if self.config.end_effector == UREndEffector.Robotiq2FingerGripper:
            gripper_slot = Slot(
                resource_name="robotiq_finger_gripper",
                resource_class="URGripper",
                capacity=1,
                attributes={
                    "gripper_type": "robotiq_finger",
                    "max_grip_force": 235.0,
                    "min_grip_position": 0,
                    "max_grip_position": 255,
                    "description": "UR Robotiq finger gripper slot",
                },
            )

            self.resource_client.init_template(
                resource=gripper_slot,
                template_name="robotiq_finger_gripper_slot",
                description="Template for UR Robotiq finger gripper slot. Used to track what the gripper is holding.",
                required_overrides=["resource_name"],
                tags=["ur", "gripper", "slot", "robotiq"],
                created_by=self.node_definition.node_id,
                version="1.0.0",
            )

            # Initialize gripper resource from template
            self.end_effector_resource_id = self.resource_client.create_resource_from_template(
                template_name="robotiq_finger_gripper_slot",
                resource_name=f"ur_gripper_{self.node_definition.node_name}",
                add_to_database=True,
            )
        elif self.config.end_effector == UREndEffector.PIPETTE:
            pipette_pool = Pool(
                resource_name="tricontinent_pipette",
                resource_class="URPipette",
                capacity=1000.0,
                attributes={
                    "pipette_type": "tricontinent",
                    "min_volume": 1.0,
                    "max_volume": 1000.0,
                    "default_speed": 150,
                    "description": "Tricontinent pipette pool for tracking tips and aspirated liquid",
                },
            )

            self.resource_client.init_template(
                resource=pipette_pool,
                template_name="tricontinent_pipette_pool",
                description="Template for Tricontinent pipette pool. Tracks pipette tips and aspirated liquids.",
                required_overrides=["resource_name"],
                tags=["ur", "pipette", "pool", "liquid-handling"],
                created_by=self.node_definition.node_id,
                version="1.0.0",
            )
            # Initialize pipette resource from template
            self.end_effector_resource_id = self.resource_client.create_resource_from_template(
                template_name="tricontinent_pipette_pool",
                resource_name=f"ur_pipette_{self.node_definition.node_name}",
                add_to_database=True,
            )

    def shutdown_handler(self) -> None:
        """Called to shutdown the node. Should be used to close connections to devices or release any other resources."""
        try:
            self.logger.log("Shutting down")
            self.integrated_controller.ur_controller.disconnect()
            self.shutdown_has_run = True
            del self.integrated_controller
            self.integrated_controller = None
            self.logger.log("Shutdown complete.")
        except Exception as err:
            self.logger.log_error(f"Error shutting down the UR Node: {err}")

    def status_handler(self):
        """Periodically called to update the current status of the node."""
        if not self.node_status.busy:
            if self.integrated_controller:
                # Getting robot state
                self.integrated_controller.ur_dashboard.get_overall_robot_status()
                movement_state, self.current_location = self.integrated_controller.ur_controller()
            else:
                self.logger.log_error("UR interface is not initialized")
                return

            if "PROTECTIVE_STOP" in self.integrated_controller.ur_dashboard.safety_status:
                self.node_status.stopped = True
                self.logger.log_error("UR is in PROTECTIVE_STOP")

            if "NORMAL" not in self.integrated_controller.ur_dashboard.safety_status:
                self.node_status.errored = True
                self.logger.log_error(f"UR ERROR: {self.integrated_controller.ur_dashboard.safety_status}")
            else:
                self.node_status.errored = False
                self.node_status.stopped = False

            if movement_state == "BUSY":
                self.node_status.busy = True
                self.logger.info("BUSY")
            elif movement_state == "READY":
                self.node_status.busy = False
        else:
            if len(self.node_status.running_actions) == 0:
                self.node_status.busy = False

    def state_handler(self) -> None:
        """Periodically called to update the current state of the node."""
        if self.integrated_controller:
            self.node_state = {
                "current_joint_angles": self.current_location,
            }

    @action()
    def move(
        self,
        target: Annotated[LocationArgument, "Linear location to move to"],
        acceleration: Annotated[Optional[float], "Acceleration"] = 0.6,
        velocity: Annotated[Optional[float], "Velocity"] = 0.6,
        linear_motion: Annotated[bool, "Use linear motion"] = True,
    ):
        """Move the robot to target location"""

        self.logger.log(f"Move location: {target.representation}")
        self.integrated_controller.ur_controller.move_to_location(
            target=target,
            acceleration=acceleration,
            velocity=velocity,
            linear_motion=linear_motion,
        )

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
    ):
        """Make a transfer using the finger gripper. This function uses linear motions to perform the pick and place movements."""

        self.gripper_pick(home, source, source_approach_axis, source_approach_distance, gripper_close)
        self.gripper_place(home, target, target_approach_axis, target_approach_distance, gripper_open)

    @action
    def gripper_pick(
        self,
        home: Annotated[LocationArgument, "Home location"],
        source: Annotated[LocationArgument, "Location to transfer sample from"],
        source_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        source_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        gripper_close: Annotated[Optional[int], "Set a min value for the gripper close state"] = 255,
    ):
        """Use the gripper to pick a piece of labware from the specified source"""
        self.logger.log_info(f"Picking from source: {source.representation}")

        self.integrated_controller.gripper_pick(
            home=home,
            source=source,
            source_approach_distance=source_approach_distance,
            source_approach_axis=source_approach_axis,
            gripper_close=gripper_close,
        )

    @action
    def gripper_place(
        self,
        home: Annotated[LocationArgument, "Home location"],
        target: Annotated[LocationArgument, "Location to transfer sample to"],
        target_approach_axis: Annotated[Optional[str], "Source location approach axis, (X/Y/Z)"] = "z",
        target_approach_distance: Annotated[Optional[float], "Approach distance in meters"] = 0.05,
        gripper_open: Annotated[Optional[int], "Set a max value for the gripper open state"] = 0,
    ):
        """Use the gripper to place a piece of labware at the target."""

        self.integrated_controller.gripper_place(
            home=home,
            target=target,
            target_approach_distance=target_approach_distance,
            target_approach_axis=target_approach_axis,
            gripper_open=gripper_open,
        )

    @action(name="e_stop", description="Emergency stop the UR robot")
    def e_stop(self):
        """Emergency stop the UR robot"""
        self.integrated_controller.ur_dashboard.power_off()
        self.logger.log_info("EMERGENCY STOP EXECUTED")

    def reset(self) -> AdminCommandResponse:
        """Reset the ur robot"""
        self.logger.log("Resetting node...")
        # If resetting startup handler does not work, try re-initializing the dashboard
        self.integrated_controller.disconnect()
        result = super().reset()
        self.logger.log("Node reset.")
        return result

    def safety_stop(self) -> AdminCommandResponse:
        """Safety stop the UR robot"""
        self.integrated_controller.ur_dashboard.stop_program()
        self.logger.log_info("SAFETY STOP EXECUTED")
        return AdminCommandResponse(success=True)


if __name__ == "__main__":
    ur_node = URNode()
    ur_node.start_node()
