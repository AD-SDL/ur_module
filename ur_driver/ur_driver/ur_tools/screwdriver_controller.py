from time import sleep
from copy import deepcopy
import numpy as np
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
            self.screwdriver = RobotiqScrewdriver(hostname=self.hostname, socket_timeout=5)
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

    def pick_screw(self, screw_loc):
        """
        Description: Picks up a new screw.
        """

        screw_approach = deepcopy(screw_loc)
        screw_approach[2] += 0.02
        screw_above = deepcopy(screw_loc)
        screw_above[2] += 0.15

        print("Picking up the screw...")
        speed_ms = 0.100

        self.ur_connection.movel(screw_above,self.accel_radss,self.speed_rads)
        sleep(2)
        speed_ms = 0.01
        self.screwdriver.activate_vacuum()
        self.ur_connection.movel(screw_approach,self.accel_radss,self.speed_rads)
        sleep(2)    
        self.ur_connection.movel(screw_loc,self.accel_mss,speed_ms)
        sleep(3)
        self.ur_connection.movel(screw_approach,self.accel_mss,speed_ms)
        sleep(2)
        speed_ms = 0.1
        self.ur_connection.movel(screw_above,self.accel_mss,speed_ms)
        sleep(2)
        if self.screwdriver.is_screw_detected() == "True":
            print("Screw successfully picked up")
        else:
            print("Failed to pick the screw")

    def screw_down(self, target):
        """
        Attempts to screws down the screw into the target location
        """
        target_approach = deepcopy(target)
        target_approach[2] += 0.02
        target_above = deepcopy(target)
        z_height = 0.15
        target_above[2] += z_height

        print("Screwing down to the target...")
        speed_ms = 0.100

        self.ur_connection.movel(target_above,self.accel_radss,self.speed_rads)
        sleep(2)
        speed_ms = 0.01
        self.ur_connection.movel(target_approach,self.accel_radss,self.speed_rads)
        sleep(2)   

        for i in np.arange(0,z_height,0.01):
            self.screwdriver.drive_clockwise(angle=10)
            self.ur_connection.translate_tool([0,0,i],0.1,0.1)
        # TODO: OR try auto screw from the exact height from the surface of the object

        # self.ur_connection.movel(target,self.accel_mss,speed_ms)
        # sleep(3)
        
        self.screwdriver.deactivate_vacuum()
        self.ur_connection.movel(target_approach,self.accel_mss,speed_ms)
        sleep(2)
        speed_ms = 0.1
        self.ur_connection.movel(target_above,self.accel_mss,speed_ms)
        sleep(2)
        if self.screwdriver.is_screw_detected() == "True":
            print("Screw successfully picked up")
        else:
            print("Failed to pick the screw")

if __name__ == "__main__":
    screwdrive = ScrewdriverController(hostname="164.54.116.129")