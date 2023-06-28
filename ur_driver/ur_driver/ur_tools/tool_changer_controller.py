from time import sleep
from copy import deepcopy

import epics

class ToolChangerController():
    

    def __init__(self, tool_changer_pv:str = None):
        self.current_tool = None
        self.pv = tool_changer_pv
        self.connect_tool_changer()
        
    def connect_tool_changer(self):
        """
        Connect tool changer
        """

        try:
            # Establishing a connection with the tool changer using EPICS library.
            self.tool_changer = epics.PV(self.pv)

        except Exception as err:
            print("Tool changer error: ", err)

        else:
            print("Tool changer is connected.")

    
    def get_tool_changer_status(self):
        """
        Description: 
            - Gets the tool changer current status. 
            - Tool changer is controlled by pyepics PV commands.
        """
        status = self.tool_changer.get()
        return status

    def lock_tool_changer(self):
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

    def unlock_tool_changer(self):
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
