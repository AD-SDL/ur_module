from time import sleep
from copy import deepcopy

class WMToolChangerController():
    """Initilizes the WMToolChangerController to pick and place tools with the Wingman tool changers"""    
    
    def __init__(self, tool_location:list = None, docking_axis:str = "-x", ur = None, tool:str = None):
        """
        """
        if not ur:
            raise AttributeError("Ur connection is not provided")
        
        self.ur = ur
        self.current_tool = None
        self.axis = docking_axis
  
        if not tool_location:
            raise Exception("Please provide a target location for the tool")
        else:
            self.location = tool_location


        self.tool_above = deepcopy(self.location)
        self.tool_above[2] += 0.05
        self.tool_front = None
        self._get_tool_front()
        self.tool_front_above = deepcopy(self.tool_front)
        self.tool_front_above[2] += 0.25

        self.robot_fast_acceleration = 1.0
        self.robot_fast_velocity = 1.0
        self.robot_slow_acceleration = 1.0
        self.robot_slow_velocity = 1.0
        
        self.gripper_com = {"baud_rate": 115200, 
                      "parity": 0,
                      "stop_bits": 1,
                      "rx_idle_chars": 1.5,
                      "tx_idle_chars": 3.5}
        
        self.pipette_com = {"baud_rate": 9600, 
                      "parity": 0,
                      "stop_bits": 1,
                      "rx_idle_chars": 1.5,
                      "tx_idle_chars": 3.5}
        
    def _set_tool_params(self,tool):
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
                
    def _set_tool_communication(self, tool:dict = None):

        if tool:
            self.ur.set_tool_communication(baud_rate=tool["baud_rate"],
                                            parity=tool["parity"],
                                            stop_bits=tool["stop_bits"],
                                            rx_idle_chars=tool["rx_idle_chars"],
                                            tx_idle_chars=tool["tx_idle_chars"]
            )

    def _get_tool_front(self):
        """
        """
        self.tool_front = deepcopy(self.location)
        if self.axis == "x":
            self.tool_front[0] += 0.1
        elif self.axis == "-x":
            self.tool_front[0] -= 0.1
        elif self.axis == "y":
            self.tool_front[1] += 0.1
        elif self.axis == "-y":
            self.tool_front[1] -= 0.1

        #TODO: Use the tool location to find which quadrant will the robot be using (x,y) and then find the 6th joint rotation to figure out which direction the robot is looking at to extract the horizatal movement axis

    def get_tool_status(self):
        """
        Description: 
            - Gets the tool changer current status. 
            - Tool changer is controlled by pyepics PV commands.
        """
        status = "STATUS"
        return status

    def pick_tool(self):
        """
        Description: 
            Picks a new tool using the tool location.
        """
        try:
            print("Picking up the tool...")
            self.ur.movel(self.tool_above, self.robot_fast_acceleration, self.robot_fast_velocity)
            self.ur.movel(self.location, self.robot_slow_acceleration, self.robot_slow_velocity)
            self.ur.movel(self.tool_front, self.robot_slow_acceleration, self.robot_slow_velocity)
            self.ur.movel(self.tool_front_above, self.robot_fast_acceleration, self.robot_fast_velocity)

        except Exception as err:
            print("Error accured while picking up the tool changer: ", err)

    def place_tool(self):
        """
        Description: 
            Places the currently attached tool back to the initial tool location
        """
        try:
            print("Placing the tool ...")
            self.ur.movel(self.tool_front_above, self.robot_fast_acceleration, self.robot_fast_velocity)
            self.ur.movel(self.tool_front, self.robot_fast_acceleration, self.robot_fast_velocity)
            self.ur.movel(self.location, self.robot_slow_acceleration, self.robot_slow_velocity)
            self.ur.movel(self.tool_above, self.robot_slow_acceleration, self.robot_slow_velocity)
        except Exception as err:
            print("Error accured while placing the tool: ", err)

    def discover_tool(self):
        """
        Discover if a tool is currently attached and which tool it is.
        """
        pass