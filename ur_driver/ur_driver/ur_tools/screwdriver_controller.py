from time import sleep
from copy import deepcopy
from .robotiq_screwdriver_driver import RobotiqScrewdriver
class ScrewdriverController():

    def __init__(self, hostname:str = None, ur_connection = None ):
        """
        """
        #TODO: Make sure interpreter urp program exsists on the polyscope then start the program using the UR Dashboard.
        #TODO: Import screwdriver driver and handle all the motions as well as screwdriving jobs here
        self.hostname = hostname
        if not ur_connection:
            raise Exception("Failed to receive UR connection!")
        else:
            self.ur_connection == ur_connection    
        self.interpreter_urp = "/path/to/iterpreter.urp"
        self.hostanem = None
        try:
            self.srewdriver = RobotiqScrewdriver(hostname=self.hostname, socket_timeout=5)
        except Exception as err:
            print(err)
            
    def check_screwdriver_controls(self):
        pass  
    def get_urp_programs(self):
        # Read from file and save it on the ur 
        pass
    def update_urp_programs(self):
        pass