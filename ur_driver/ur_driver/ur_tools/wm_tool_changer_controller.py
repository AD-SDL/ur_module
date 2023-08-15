from time import sleep
from copy import deepcopy

import epics

class WMToolChangerController():
    """Initilizes the WMToolChangerController to pick and place tools with the Wingman tool changers"""    
    
    def __init__(self, tool_location:list = None, horizontal_axis:str = "y", ur_connection = None):
        """
        """
        if not ur_connection:
            raise AttributeError("Ur connection is not provided")
        
        self.robot = ur_connection
        self.current_tool = None
        self.axis = horizontal_axis

        self.tool_dock_l = [-0.30533163571362804, 0.293042569973924, 0.234306520730365, -3.1414391023029085, 0.014564845435757333, 0.0040377171549781125]
        self.tool_dock_j = [2.692000389099121, -2.031496664086813, -1.3187065124511719, -1.3587687772563477, -4.709893051777975, 1.129366159439087]
        self.tool_above_j = [2.691868543624878, -2.01027836422109, -1.1145529747009277, -1.584151407281393, -4.710240427647726, 1.1290664672851562]
        self.tool_above_l = [-0.30521326850056485, 0.29304507597600077, 0.2842680699923231, -3.141487582034991, 0.014625666717814146, 0.0039104466707534005]

        if tool_location:
            self.location = tool_location
        else:
            self.location = self.tool_dock_l

        self.tool_above = self.location
        self.tool_above[2] += 0.05
        self.tool_front = None
        self._get_tool_front()

    def _get_tool_front(self):
        """
        """
        self.tool_front = self.location
        if self.axis == "x":
            self.tool_front[0] += 0.1
        elif self.axis == "-x":
            self.tool_front[0] -= 0.1
        elif self.axis == "y":
            self.tool_front[1] += 0.1
        elif self.axis == "-y":
            self.tool_front[1] -= 0.1

        #TODO: USe the tool location to find which quadrant will the robot be using (x,y) and then find the 6th joint rotation to figure out which direction the robot is looking at to extract the horizatal movement axis

    def get_tool_status(self):
        """
        Description: 
            - Gets the tool changer current status. 
            - Tool changer is controlled by pyepics PV commands.
        """
        status = "STATUS"
        return status

    def pick_tool(self ):
        """
        Description: 
            Picks a new tool using the tool location.
        """
        try:
            print("Picking up the tool...")
            tool_above 
            self.robot.movel(self.location)
        except Exception as err:
            print("Error accured while picking up the tool changer: ", err)

    def place_tool(self):
        """
        Description: 
            Places the currently attached tool back to the initial tool location
        """
        try:
            print("Placing the tool ...")
            self.tool_changer.put(0)
        except Exception as err:
            print("Error accured while unlocking the tool changer: ", err)

    def discover_tool(self):
        """
        Discover if a tool is currently attached and which tool it is.
        """
        pass