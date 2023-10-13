from time import sleep
from copy import deepcopy

class ScrewdriverController():

    def __init__(self ):
        """
        """
        #TODO: Make sure interpreter urp program exsists on the polyscope then start the program using the UR Dashboard.
        #TODO: Import screwdriver driver and handle all the motions as well as screwdriving jobs here
            
        self.interpreter_urp = "/path/to/iterpreter.urp"
        self.hostanem = None
        self.ur_connection = None
        self.srewdriver = None
    def check_screwdriver_controls(self):
        pass  
    def get_urp_programs(self):
        # Read from file and save it on the ur 
        pass
    def update_urp_programs(self):
        pass