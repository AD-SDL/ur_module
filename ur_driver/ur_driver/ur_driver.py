#!/usr/bin/env python3

import threading
import socket 

from multiprocessing.connection import wait
from time import sleep
from copy import deepcopy
import json

from ur_dashboard import UR_DASHBOARD
from ur_tools import *
from urx import Robot, RobotException

class Connection():
    """Connection to the UR robot to be shared within UR driver """
    def __init__(self,  hostname:str = "146.137.240.38", PORT: int = 29999) -> None:

        self.hostname = hostname
        self.PORT = PORT
        
        self.connection = None
        self.connect_ur()

    def connect_ur(self):
        """
        Description: Create conenction to the UR robot
        """

        for i in range(10):
            try:
                self.connection = Robot(self.hostname)

            except socket.error:
                print("Trying robot connection ...")
                sleep(10)

            else:
                print('Successful ur connection')
                break

    def disconnect_ur(self):
        """
        Description: Disconnects the socket connection with the UR robot
        """
        self.connection.close()
        print("Robot connection is closed.")

class UR(UR_DASHBOARD):
    """
    This is the primary class for UR robots. 
    It integrates various interfaces to achieve comprehensive control, encompassing robot initialization via the UR dashboard, 
    robot motion using URx, and the management of robot end-effectors such as grippers, screwdrivers, electronic pipettes, and cameras."    
    """    
    def __init__(self, hostname:str = None, PORT: int = 29999):
        """Constructor for the UR class.
        :param hostname: Hostname or ip.
        :param port: Port.
        """

        if not hostname:
            raise TypeError("Hostname cannot be None Type!")
        
        super().__init__(hostname=hostname, PORT=PORT)

        self.hostname = hostname
        self.PORT = PORT
        self.ur = Connection(hostname = self.hostname, PORT = self.PORT)
        self.ur_connection = self.ur.connection
        self.ur_connection.set_tcp((0, 0, 0, 0, 0, 0))
        self.acceleration = 0.5
        self.velocity = 0.5
        self.speed_ms    = 0.750
        self.speed_rads  = 0.750
        self.accel_mss   = 1.200
        self.accel_radss = 1.200
        self.blend_radius_m = 0.001
        self.ref_frame = [0,0,0,0,0,0]
        self.robot_current_joint_angles = None
        self.get_movement_state()
        #TODO: get the information of what is the current tool attached to UR. Maybe keep the UR unattached after the tools were used? Run a senity check at the beginning to findout if a tool is connected 
    
    def get_movement_state(self):
        current_location = self.ur_connection.getj()
        current_location = [ '%.2f' % value for value in current_location] #rounding to 3 digits
        # print(current_location)
        if self.robot_current_joint_angles == current_location:
            movement_state = "READY"
        else:
            movement_state = "BUSY"

        self.robot_current_joint_angles = current_location

        return movement_state

    def home(self, home_location = None):
        """
        Description: Moves the robot to the home location.
        """
        print("Homing the robot...")
        if home_location:
            home_loc = home_location
        else:
            home_loc = [-1.355567757283346, -2.5413090191283167, 1.8447726408587855, -0.891581193809845, -1.5595606009112757, 3.3403327465057373]
        self.ur_connection.movej(home_loc, self.acceleration + 0.5, self.velocity + 0.5, 0, 0)
        sleep(3.5)

        print("Robot homed")

    def gripper_transfer(self, pos1, pos2, gripper_rotation:str = None, safe_heigh: int = None):
        '''
        Make a transfer using the finger gripper
        ''' 
        gripper_controller = FingerGripperController(hostname = self.hostname, ur_connection = self.ur_connection)
        gripper_controller.connect_gripper()
        # robot.ur_connection.set_payload(2, (0, 0, 0.1))

        gripper_controller.pick(pos1)
        gripper_controller.place(pos2)
        print('Finished transfer')
        gripper_controller.disconnect_gripper()

    def pick_tool(self, home, tool_loc, docking_axis = "y", payload = 0.12):
        """
            Picks up a tool using the given tool location
        """
        self.ur_connection.set_payload(payload)
        wingman_tool = WMToolChangerController(tool_location = tool_loc, docking_axis = docking_axis, ur_connection = self.ur_connection)
        self.home(home)
        wingman_tool.pick_tool()
        self.home(home)    

    def place_tool(self, home, tool_loc, docking_axis = "y"):
        """
            Picks up a tool using the given tool location
        """
        wingman_tool = WMToolChangerController(tool_location = tool_loc, docking_axis = docking_axis, ur_connection = self.ur_connection)
        self.home(home)
        wingman_tool.place_tool()
        self.home(home)    
    
    def run_droplet(self, home, tip_loc, sample_loc, droplet_loc, tip_trash):
        """Create droplet"""

        pipette = ApsPipetteController(ur_connection = self.ur_connection, IP = self.hostname)
        pipette.connect_pipette()

        self.home(home)
        pipette.pick_tip(tip_loc=tip_loc)
        pipette.transfer_sample(sample_loc=sample_loc)
        self.home(home)
        pipette.create_droplet(droplet_loc=droplet_loc)
        self.home(home)
        pipette.empty_tip(sample_loc=sample_loc)     
        pipette.eject_tip(eject_tip_loc=tip_trash)
        self.home(home)
        pipette.disconnect_pipette()

    def run_urp_program(self, transfer_file_path:str = None, program_name: str = None):

        """"""
        if not program_name:
            raise ValueError("Provide program name!")
        
        ur_program_path = "/programs/" + program_name 

        if transfer_file_path:
            self.transfer_program(local_path = transfer_file_path, ur_path = ur_program_path)
            sleep(2)

        self.load_program(program_path = ur_program_path)
        sleep(2)
        self.run_program()
        sleep(5)
        
        print("Running the URP program: ", program_name)
        time_elapsed = 0
        program_err = ""
        
        program_status = "BUSY"
        ready_status_count = 0
        while program_status == "BUSY":
            if self.get_movement_state() == "READY":
                ready_status_count += 1
                if ready_status_count >=6:
                    program_status = "READY"
            else:
                ready_status_count = 0
            sleep(3)

        #TODO: FIX the output loggings 

        # if "STOPPED" in program_state:       
        program_log = {"output_code":"0", "output_msg": "Successfully finished " + program_name, "output_log": "seconds_elapsed:" + str(time_elapsed)}
        # elif "PAUSED" in program_state:
            # program_log = {"output_code":"-1", "output_msg": "Failed running: " + program_name, "output_log": program_err}
        # else:
            # program_log = {"output_code":"-1", "output_msg": "Unkown program state:  " + program_name, "output_log": program_state}

        return program_log


if __name__ == "__main__":

    pos1= [-0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    pos2= [0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    robot = UR(hostname="164.54.116.129")
    # print(robot.get_joint_angles())
    tool_loc = [0.32704628917562345, -0.1017379678362813, 0.3642503117806354, -2.1526354130031917, 2.2615882459741723, -0.04632031979240964]
    home = [0.5431541204452515, -1.693524023095602, -0.7301170229911804, -2.2898713550963343, 1.567720651626587, -1.0230830351458948]
    tip1 = [0.04639965460538513, 0.4292986855073111, 0.0924689410052111, -3.1413810571577048, 0.014647332926328135, 0.004028900798665303]
    sample = [0.07220331720579347, 0.21138438053671288, 0.11898933185468973, -3.141349185677643, 0.014592306794949944, 0.004077757329820521]
    droplet = [-0.21435167102697, 0.31117471247776396, 0.273829131948966, 3.126800328499299, -0.017429873171790906, -0.007516422536326644]
    tip_eject = [0.10270290312926324, 0.2862487614384484, 0.10155903555930355, 3.1268372121718597, -0.01760209112687455, -0.007607510036297549]
    pipette_loc = [0.21285670041158733, 0.1548897634390196, 0.005543999069077835, 3.137978068966478, -0.009313836267512065, -0.0008972976992386885]
    handE_loc = [0.3131286590368134, 0.15480163498252172, 0.005543999069077835, 3.137978068966478, -0.009313836267512065, -0.0008972976992386885]
    screwdriver_loc = [0.43804370307762014, 0.15513117190281586, 0.006677533813616729, 3.137978068966478, -0.009313836267512065, -0.0008972976992386885]
    target = [0.24769823122656057, -0.3389885625301465, 0.368077779916273, 2.1730827596713733, -2.264911265531878, 0.0035892213555669857]
    cell_screw = [0.24930985075448253, -0.24776717616652696, 0.4181221227946348, 3.039003299245514, -0.7400526434644932, 0.016640327870615954]
    screw_holder = [0.20689856249907082, -0.30748395554908325, 0.39225140260522395, 3.1256340911820044, -0.009252195445917084, 0.026416395536066287]
    # robot.home(home)
    # robot.pick_tool(home, pipette_loc,payload=1.2)
    # robot.ur_connection.movel(target,1,1)
    # sleep(1)
    # robot.ur_driver/ur_driver/ur_driver.py(home,pipette_loc)
    # robot.pick_tool(home, handE_loc,payload=1.2)
    # robot.ur_connection.movel(target,1,1)
    # sleep(1)
    gripper_controller = FingerGripperController(hostname = robot.hostname, ur_connection = robot)
    gripper_controller.connect_gripper()
    gripper_controller.gripper.move_and_wait_for_pos(255, 255, 255)
    # robot.place_tool(home,handE_loc)


    # robot.home(home)
    # robot.pick_tool(home, screwdriver_loc,payload=3)
    # sr = ScrewdriverController(hostname=robot.hostname, ur_connection=robot)
    # sr.screwdriver.activate_screwdriver()
    # sr.pick_screw(screw_holder)
    # sr.screw_down(cell_screw)
    # robot.home(home)
    # robot.place_tool(home,screwdriver_loc)
  
    
    robot.ur.disconnect_ur()





