from copy import deepcopy
from typing import Union

from madsci.common.types.location_types import LocationArgument

from ur_interface.ur_tools.pipettes.abstract_pipette_interfaces import Pipette


class PipetteMixin:
    def pipette_aspirate_from_source_location(
        self,
        volume: float,
        source: Union[LocationArgument, list],
        home: Union[LocationArgument, list, None] = None,
        approach_axis: str = None,
        approach_distance: float = None,
    ):
        """Pick up from source position"""
        if not isinstance(self.end_effector, Pipette):
            raise Exception("End-effector is not a pipette, cannot perform aspirate operation")
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

        self.end_effector.aspirate(volume)

        self.logger.debug("Moving back to above goal position")
        self.ur_controller.move_to_location(above_goal, linear_motion=True)
        self.logger.info("Pick operation completed successfully")
        # if self.resource_client is not None:
        #     object, _ = self.resource_client.pop(source_location.resource_id)
        #     self.resource_client.push(self.end_effector_resource_id, object)

    def pipette_dispense_to_target_location(
        self,
        volume: float,
        target: Union[LocationArgument, list],
        home: Union[LocationArgument, list, None] = None,
        approach_axis: str = None,
        approach_distance: float = None,
    ):
        """Pick up from source position"""
        if not isinstance(self.end_effector, Pipette):
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

       

        self.logger.debug("Moving to above goal position")
        self.ur_controller.move_to_location(above_goal, linear_motion=True)

        self.logger.debug("Moving to goal position")
        self.ur_controller.move_to_location(target_location, linear_motion=True)

        self.end_effector.dispense(volume)
        if self.resource_client is not None:
            object, _ = self.resource_client.pop(target_location.resource_id)
            self.resource_client.push(self.end_effector_resource_id, object)

        self.logger.debug("Moving back to above goal position")
        self.ur_controller.move_to_location(above_goal, linear_motion=True)
        self.logger.info("Pick operation completed successfully")

    def pipette_transfer(
        self,
        volume: float,
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
        self.pipette_aspirate_from_source_location(
            volume=volume,
            source=source,
            home=home,
            approach_axis=source_approach_axis,
            approach_distance=source_approach_distance,
        )
        self.logger.info("Pick completed")

        self.pipette_dispense_to_target_location(
            volume=volume,
            target=target,
            home=home,
            approach_axis=target_approach_axis,
            approach_distance=target_approach_distance,
        )
        self.logger.info("Place completed")
