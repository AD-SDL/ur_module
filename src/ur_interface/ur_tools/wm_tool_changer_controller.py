"""WingMan Tool Changer Controller Class"""

from copy import deepcopy
from time import sleep
from typing import Union

from madsci.common.types.location_types import LocationArgument


class WMToolChangerController:
    """Initilizes the WMToolChangerController to pick and place tools with the Wingman tool changers"""

    def __init__(
        self,
        tool_location: Union[LocationArgument, list] = None,
        docking_axis: str = "y",
        ur=None,
        tool: str = None,
        resource_client=None,
        tool_resource_id: str = None,
    ):
        """Constractor class

        Args:
            tool_location (list): Tool location
            docking axis (str): Docking axis based on the robot location
            ur (urx.Robot): UR robot connection
            tool (str): Tool name
        """
        if not ur:
            raise AttributeError("Ur connection is not provided")

        self.ur = ur
        self.resource_client = resource_client
        self.tool_resource_id = tool_resource_id
        self.tool = tool
        self.current_tool = None
        self.axis = docking_axis

        if not tool_location:
            raise Exception("Please provide a target location for the tool")
        else:
            self.location = tool_location

        self._set_target_locations()
        self.robot_fast_acceleration = 1.0
        self.robot_fast_velocity = 1.0
        self.robot_slow_acceleration = 0.2
        self.robot_slow_velocity = 0.2

        self.gripper_com = {
            "baud_rate": 115200,
            "parity": 0,
            "stop_bits": 1,
            "rx_idle_chars": 1.5,
            "tx_idle_chars": 3.5,
        }

        self.pipette_com = {
            "baud_rate": 9600,
            "parity": 0,
            "stop_bits": 1,
            "rx_idle_chars": 1.5,
            "tx_idle_chars": 3.5,
        }

    def _set_target_locations(self):
        """Sets the target locations for the tool changer"""
        if isinstance(self.location, LocationArgument):
            self.location_joint_values = self.location.location
        elif isinstance(self.location, list):
            self.location_joint_values = self.location

        self.tool_above = deepcopy(self.location_joint_values)
        self.tool_above[2] += 0.05
        self.tool_front = None
        self._get_tool_front()
        self.tool_front_above = deepcopy(self.tool_front)
        self.tool_front_above[2] += 0.25

    def _set_tool_params(self, tool: str = None):
        """Sets Tool Parameters

        Args:
            tool (str): Tool name
        """
        if tool:
            if tool.lower() == "gripper":
                self._set_tool_communication(tool=self.gripper_com)
                sleep(4)
                if self.discover_tool():
                    pass
                else:
                    pass

            elif tool.lower() == "pipette":
                self._set_tool_communication(tool=self.pipette_com)
                sleep(4)
                if self.discover_tool():
                    pass
                else:
                    pass

    def _set_tool_communication(self, tool: dict = None):
        """Sets tool communication using the tool specific parameters

        Args:
            tool (dict): tool specific parameters to set the correct Tool I/O
        """
        if tool:
            self.ur.set_tool_communication(
                baud_rate=tool["baud_rate"],
                parity=tool["parity"],
                stop_bits=tool["stop_bits"],
                rx_idle_chars=tool["rx_idle_chars"],
                tx_idle_chars=tool["tx_idle_chars"],
            )

    def _get_tool_front(self):
        """Gets the tool front location"""
        self.tool_front = deepcopy(self.location_joint_values)
        if self.axis == "x":
            self.tool_front[0] += 0.1
        elif self.axis == "-x":
            self.tool_front[0] -= 0.1
        elif self.axis == "y":
            self.tool_front[1] += 0.1
        elif self.axis == "-y":
            self.tool_front[1] -= 0.1

        # TODO: Use the tool location to find which quadrant will the robot be using (x,y) and then find the 6th joint rotation to figure out which direction the robot is looking at to extract the horizatal movement axis

    def get_tool_status(self):
        """Gets the tool changer current status.

        Returns (str): Status string
        """
        status = "STATUS"
        return status

    def pick_tool(self):
        """Picks a new tool using the tool location."""
        try:
            print("Picking up the tool...")
            self.ur.movel(
                self.tool_above,
                self.robot_fast_acceleration,
                self.robot_fast_velocity,
            )
            self.ur.movel(
                self.location_joint_values,
                self.robot_slow_acceleration,
                self.robot_slow_velocity,
            )
            self.ur.movel(
                self.tool_front,
                self.robot_slow_acceleration,
                self.robot_slow_velocity,
            )
            self.ur.movel(
                self.tool_front_above,
                self.robot_fast_acceleration,
                self.robot_fast_velocity,
            )

        except Exception as err:
            print(
                "Error accured while picking up the tool changer: ",
                err,
            )

    def place_tool(self):
        """Places the currently attached tool back to the initial tool location"""
        try:
            print("Placing the tool ...")
            self.ur.movel(
                self.tool_front_above,
                self.robot_fast_acceleration,
                self.robot_fast_velocity,
            )
            self.ur.movel(
                self.tool_front,
                self.robot_fast_acceleration,
                self.robot_fast_velocity,
            )
            self.ur.movel(
                self.location_joint_values,
                self.robot_slow_acceleration,
                self.robot_slow_velocity,
            )
            self.ur.movel(
                self.tool_above,
                self.robot_slow_acceleration,
                self.robot_slow_velocity,
            )
        except Exception as err:
            print(
                "Error accured while placing the tool: ",
                err,
            )

    def discover_tool(self):
        """Discover if a tool is currently attached and which tool it is."""
        ### Check resources DB for the tool
        pass
