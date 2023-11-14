import os

from time import sleep
from copy import deepcopy
import numpy as np

from .robotiq_screwdriver_driver import RobotiqScrewdriver

class ScrewdriverController():

    def __init__(self, hostname:str = None, ur = None, ur_dashboard = None):
        """
        """
        #TODO: Make sure interpreter urp program exsists on the polyscope then start the program using the UR Dashboard.
        #TODO: Import screwdriver driver and handle all the motions as well as screwdriving jobs here
       
        self.hostname = hostname
        self.ur = None
        self.air_switch_digital_output = 0

        current_dir = os.getcwd()
        index = current_dir.find("ur_module")
        parent_dir = current_dir[:index+10]
        self.interpreter_urp =  parent_dir + "/ur_driver/scripts/urp_programs/interpreter_mode.urp"

        if not ur:
            raise Exception("Failed to receive UR connection!")
        else:
            self.ur_dashboard = ur_dashboard
            self.ur = ur
            self.ur.set_payload(3)
        
        try:
            self.screwdriver = RobotiqScrewdriver(hostname=self.hostname, socket_timeout=5)
            self.screwdriver.connect()
        except Exception as err:
            print(err)
        
        self.load_interpreter_socket_program()
        
    def load_interpreter_socket_program(self):
        """
        Makes sure that the interpreter socket is enabled on the robot PolyScope so that screwdriver commands can be sent over this socket.
        """
        iterpreter_program =  "/programs/interpreter_mode.urp"
        response = self.ur_dashboard.load_program(iterpreter_program)
        if "File not found" in response:
            self.ur_dashboard.transfer_program(local_path = self.interpreter_urp, ur_path = iterpreter_program)
            response = self.ur_dashboard.load_program(iterpreter_program)
        self.ur_dashboard.run_program()
        sleep(2)

    def pick_screw(self, screw_loc:list = None, approach_axis:str = "z", approach_distance:float = 0.02):
        """
        Description: Picks up a new screw.
        """
        if not screw_loc:
            raise Exception("Please provide the source loaction")
        
        axis = None

        if not approach_axis or approach_axis.lower() == "z":
            axis = 2
        elif approach_axis.lower() == "y":
            axis = 1
        elif approach_axis.lower() == "-y":
            axis = 1
            approach_distance = -approach_distance
        elif approach_axis.lower() == "x":
            axis = 0
        elif approach_axis.lower() == "-x":
            axis = 0
            approach_distance = -approach_distance

        screw_above = deepcopy(screw_loc)
        screw_above[axis] += approach_distance
        screw_start_loc = deepcopy(screw_loc)
        screw_start_loc[axis] += 0.01

        print("Picking up the screw...")
        
        self.ur.movel(screw_above,1,1)
        self.ur.movel(screw_start_loc,0.5,0.5)
        self.ur.set_digital_out(self.air_switch_digital_output, True)
        self.ur_dashboard.run_program() #Restart interpreter program
        sleep(2)
        self.screwdriver.activate_vacuum()
        self.screwdriver.auto_screw()
        sleep(4)    
        self.ur.movel(screw_above,1,0.5)
        sleep(2)

    def screw_down(self, target:list = None, approach_axis:str = "z", approach_distance:float = 0.02):
        """
        Attempts to screws down the screw into the target location
        """
        if not target:
            raise Exception("Please provide the target loaction")
        
        axis = None

        if not approach_axis or approach_axis.lower() == "z":
            axis = 2
        elif approach_axis.lower() == "y":
            axis = 1
        elif approach_axis.lower() == "-y":
            axis = 1
            approach_distance = -approach_distance
        elif approach_axis.lower() == "x":
            axis = 0
        elif approach_axis.lower() == "-x":
            axis = 0
            approach_distance = -approach_distance

        target_above = deepcopy(target)
        z_height = approach_distance
        target_above[axis] += z_height

        print("Screwing down to the target...")
        sleep(1)
        self.ur.movel(target_above,1,1)
        self.ur.set_digital_out(self.air_switch_digital_output, True)

        self.ur.movel(target,1,1)
        sleep(1)
        self.ur_dashboard.run_program() #Restart interpreter program
        sleep(2)
        self.screwdriver.activate_vacuum()
        self.screwdriver.auto_screw(250)
        sleep(2)
        self.screwdriver.drive_clockwise(angle=200,rpm=100)
        sleep(2)
        self.screwdriver.deactivate_vacuum()
        self.ur.set_digital_out(self.air_switch_digital_output, False)
        sleep(1)
        self.ur.movel(target_above,0.5,0.5)
        sleep(2)
        # self.ur_dashboard.run_program() #Restart interpreter program
        # sleep(2)
        print("Screw successfully placed")

        # if self.screwdriver.is_screw_detected() == "False":
        #     print("Screw successfully placed")
        #     # self.screwdriver.deactivate_vacuum()
        # else:
        #     print("Failed to place the screw")

    def transfer(self, source:list = None, target:list = None, source_approach_axis:str = None, target_approach_axis:str = None, source_approach_dist:float = None, target_approach_dist:float = None) -> None:
        """Handles a screw transfer"""

        self.pick_screw(screw_loc = source, approach_axis = source_approach_axis, approach_distance = source_approach_dist)
        print("Pick screw completed")
        self.screw_down(target=target, approach_axis = target_approach_axis, approach_distance = target_approach_dist)
        print("Place screw completed")

if __name__ == "__main__":
    screwdrive = ScrewdriverController(hostname="164.54.116.129")