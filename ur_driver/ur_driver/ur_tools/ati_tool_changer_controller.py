from time import sleep
from copy import deepcopy

import epics

class WMToolChangerController():
    """Initilizes the WMToolChangerController to pick and place tools with the Wingman tool changers"""    
    
    def __init__(self, tool_location:str = None):
        self.current_tool = None
        self.location = tool_location
        self.connect_tool_changer()
        
    def get_tool_status(self):
        """
        Description: 
            - Gets the tool changer current status. 
            - Tool changer is controlled by pyepics PV commands.
        """
        status = self.tool_changer.get()
        return status

    def pick_tool(self, ):
        """
        Description: 
            - Locks the tool changer. 
            - Tool changer is controlled by pyepics PV commands.
        """
        try:
            print("Locking the tool changer...")
            self.tool_changer.put(1)
        except Exception as err:
            print("Error accured while locking the tool changer: ", err)

    def place_tool(self):
        """
        Description: 
            - Unlocks the tool changer. 
            - Tool changer is controlled by pyepics PV commands.
        """
        try:
            print("Unlocking the tool changer...")
            self.tool_changer.put(0)
        except Exception as err:
            print("Error accured while unlocking the tool changer: ", err)

    def discover_tool(self):
        """
        Discover if a tool is currently attached and which tool it is.
        """
        pass