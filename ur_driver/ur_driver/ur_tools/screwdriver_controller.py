import os

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
        self.air_switch_digital_output = 0

        current_dir = os.getcwd()
        index = current_dir.find("ur_module")
        parent_dir = current_dir[:index+10]
        self.interpreter_urp =  parent_dir + "/ur_driver/scripts/urp_programs/interpreter_mode.urp"

        if not ur_connection:
            raise Exception("Failed to receive UR connection!")
        else:
            self.ur_connection = ur_connection
            self.ur_connection.ur_connection.set_payload(3)
        
        try:
            self.screwdriver = RobotiqScrewdriver(hostname=self.hostname, socket_timeout=5)
            self.screwdriver.connect()
        except Exception as err:
            print(err)
        
        self.load_interpreter_socket_program()

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
        response = self.ur_connection.load_program(iterpreter_program)
        if "File not found" in response:
            self.ur_connection.transfer_program(local_path = self.interpreter_urp, ur_path = iterpreter_program)
            response = self.ur_connection.load_program(iterpreter_program)
        self.ur_connection.run_program()
        sleep(2)
        # Bug: Robot powers off after loading the program and needs a reboot. Check loaded program status and  make sure it works before moving to next steps

    def pick_screw(self, screw_loc):
        """
        Description: Picks up a new screw.
        """

        screw_above = deepcopy(screw_loc)
        screw_above[2] += 0.035
        screw_approach = deepcopy(screw_loc)
        screw_approach[2] += 0.01

        print("Picking up the screw...")
        
        self.ur_connection.ur_connection.movel(screw_above,1,1)
        # sleep(2)
        self.ur_connection.ur_connection.movel(screw_approach,0.5,0.5)
        # sleep(1)
        self.ur_connection.run_program() #Restart interpreter program
        sleep(2)
        self.ur_connection.set_digital_out(self.air_switch_digital_output, True)
        self.screwdriver.activate_vacuum()
        self.screwdriver.auto_screw()
        sleep(4)    
        # self.screwdriver.connection.execute_command("movel(p{},a={},v={},r={})".format(screw_above,0.5,0.5,0,0))

        self.ur_connection.ur_connection.movel(screw_above,1,0.5)
        sleep(2)
        self.ur_connection.run_program() #Restart interpreter program
        sleep(2)
        if self.screwdriver.is_screw_detected() == "True":
            print("Screw successfully picked up")
        else:
            print("Failed to pick the screw")

    def screw_down(self, target):
        """
        Attempts to screws down the screw into the target location
        """

        target_above = deepcopy(target)
        z_height = 0.01
        target_above[2] += z_height

        print("Screwing down to the target...")
        sleep(1)
        self.ur_connection.ur_connection.movel(target_above,1,1)
        self.ur_connection.run_program() #Restart interpreter program
        # self.screwdriver.connection.execute_command("movel(p{},a={},v={},r={})".format(target_above,0.5,0.5,0,0))
        sleep(2)
        self.screwdriver.activate_vacuum()
        self.screwdriver.auto_screw(150)
        sleep(2)
        self.screwdriver.deactivate_vacuum()
        sleep(1)
        self.ur_connection.ur_connection.movel(target_above,0.5,0.5)
        sleep(2)
        # self.ur_connection.run_program() #Restart interpreter program
        # sleep(2)
        print("Screw successfully placed")

        # if self.screwdriver.is_screw_detected() == "False":
        #     print("Screw successfully placed")
        #     # self.screwdriver.deactivate_vacuum()
        # else:
        #     print("Failed to place the screw")


if __name__ == "__main__":
    screwdrive = ScrewdriverController(hostname="164.54.116.129")