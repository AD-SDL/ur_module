from time import sleep
from copy import deepcopy
from .robotiq_screwdriver_driver import RobotiqScrewdriver
class ScrewdriverController():

    def __init__(self, hostname:str = None, ur_connection = None):
        """
        """
        #TODO: Make sure interpreter urp program exsists on the polyscope then start the program using the UR Dashboard.
        #TODO: Import screwdriver driver and handle all the motions as well as screwdriving jobs here
       
        self.hostname = hostname
        self.ur_connection = None
        self.interpreter_urp =  "../../scripts/urp_programs/interpreter_mode.urp"

        if not ur_connection:
            raise Exception("Failed to receive UR connection!")
        else:
            self.ur_connection == ur_connection    
        
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

    def load_interpreter_socket_program(self):
        """
        Makes sure that the interpreter socket is enabled on the robot PolyScope so that screwdriver commands can be sent over this socket.
        """
        iterpreter_program =  "/programs/interpreter_mode.urp"
        response = self.ur_connection.load_program("iterpreter_program")
        if "File not found" in response:
            self.ur_connection.transfer_program(local_path = self.interpreter_urp, ur_path = iterpreter_program)
            response = self.ur_connection.load_program("iterpreter_program")
        self.ur_connection.run_program()
    def
if __name__ == "__main__":
    screwdrive = ScrewdriverController(hostname="164.54.116.129")