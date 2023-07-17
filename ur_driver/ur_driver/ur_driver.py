#!/usr/bin/env python3

import threading
import socket 

from multiprocessing.connection import wait
from time import sleep
from copy import deepcopy
import json

from ur_driver.ur_dashboard import UR_DASHBOARD
from .ur_tools import *
from urx import Robot, RobotException


class Connection():
    """Connection to the UR robot to be shared within UR driver """
    def __init__(self,  IP:str = "146.137.240.38", PORT: int = 29999) -> None:

        self.IP = IP
        self.PORT = PORT
        
        self.connection = None
        self.connect_ur()

    def connect_ur(self):
        """
        Description: Create conenction to the UR robot
        """

        for i in range(10):
            try:
                self.connection = Robot(self.IP)

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
    

    def __init__(self, IP:str = "146.137.240.38", PORT: int = 29999):
        
        # if not connection:
        #     raise Exception("Robot connection is not established")
        # else:
        #     self.ur = connection
        super().__init__(IP=IP, PORT=PORT)

        self.IP = IP
        self.PORT = PORT
        
        self.ur_connection = Connection(IP = self.IP, PORT = self.PORT).connection
        
        self.acceleration = 0.5
        self.velocity = 0.2
        self.speed_ms    = 0.750
        self.speed_rads  = 0.750
        self.accel_mss   = 1.200
        self.accel_radss = 1.200
        self.blend_radius_m = 0.001
        self.ref_frame = [0,0,0,0,0,0]

        self.robot_current_joint_angles = None
        self.get_movement_state()
        #TODO: get the information of what is the current tool attached to UR. Maybe keep the UR unattached after the tools were used? Run a senity check at the beginning to findout if a tool is connected 

    def get_joint_angles(self):
        
        return self.ur_connection.getj()
    
    def get_cartesian_coordinates(self):
        
        return self.ur_connection.getl()
    
    def get_movement_state(self):
        current_location = self.get_joint_angles()
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
        self.ur_connection.movej(home_loc, self.acceleration, self.velocity, 0, 0)
        sleep(4)

        print("Robot moved to home location")

    def pick_pipette(self):

        pipette_controller = PipetteController(ur_connection=self.ur_connection)
        tool_changer_controller = ToolChangerController()

        pipette_controller.move_pipette_dock()
        tool_changer_controller.lock_tool_changer()
        pipette_controller.lift_pipette_on_dock()

    def place_pipette(self):

        pipette_controller = PipetteController(ur_connection=self.ur_connection)
        tool_changer_controller = ToolChangerController()

        pipette_controller.move_pipette_dock()
        tool_changer_controller.unlock_tool_changer()
        pipette_controller.lift_pipette_on_dock()  
        pipette_controller.disconnect_pipette()
        tool_changer_controller.disconnect_tool_changer()

    def create_sample(self, home = None, sample1_loc = None, sample2_loc = None, well_loc = None, tip1_loc = None, tip2_loc = None):
        """"""
        if home:
            home_J = home
        else:
            home_J = [2.017202138900757, -1.137721137409546, -0.9426093101501465, -2.6425615749754847, -4.693090263997213, -3.8424256483661097]

        pipette_controller = PipetteController(ur_connection=self.ur_connection)
        pipette_controller.connect_pipette()
        self.home(home_J)
        pipette_controller.pick_tip(tip_loc = tip1_loc)
        pipette_controller.transfer_sample(sample_loc = sample1_loc)
        pipette_controller.drop_tip_to_trash()
        pipette_controller.pick_tip(tip_loc = tip2_loc)
        pipette_controller.transfer_sample(sample_loc = sample2_loc)
        pipette_controller.mix_samples(well_loc = well_loc)
        self.home(home_J)
        pipette_controller.disconnect_pipette()

    def run_droplet(self):
        pipette_controller = PipetteController(ur_connection=self.ur_connection)
        pipette_controller.create_droplet()
        pipette_controller.retrieve_droplet()
        pipette_controller.disconnect_pipette()

    def dispose_tip(self):
        pipette_controller = PipetteController(ur_connection=self.ur_connection)
        home_J = [2.017202138900757, -1.137721137409546, -0.9426093101501465, -2.6425615749754847, -4.693090263997213, -3.8424256483661097]
        self.home(home_J)
        pipette_controller.empty_tip()
        pipette_controller.drop_tip_to_trash()
        self.home(home_J)
        pipette_controller.disconnect_pipette()

    def droplet_exp(self, tip_number_1:int = None, tip_number_2:int = None):
        """
        DEPRECATED
        Description: Runs the full droplet experiment by calling the functions that perform each step in the experiment.
        """
        print("-*-*-* Starting the droplet experiment *-*-*-")
        # home_J = [2.017202138900757, -1.137721137409546, -0.9426093101501465, -2.6425615749754847, -4.693090263997213, -3.8424256483661097]

        # pipette_controller = PipetteController(ur_connection=self.ur_connection)
        # tool_changer_controller = ToolChangerController()

        # pipette_controller.move_pipette_dock()
        # tool_changer_controller.lock_tool_changer()
        # pipette_controller.lift_pipette_on_dock()    

        # self.home(home_J)
        # pipette_controller.empty_tip()
        # pipette_controller.drop_tip_to_trash()
        # self.home(home_J)

        # pipette_controller.move_pipette_dock()
        # tool_changer_controller.unlock_tool_changer()
        # pipette_controller.lift_pipette_on_dock()         
        print("-*-*-* Droplet experiment is completed *-*-*-")
        
    def gripper_transfer(self, pos1, pos2, gripper_rotation:str = None, safe_heigh: int = None):
        '''
        Make a transfer using the finger gripper
        ''' 
        self.ur_connection.set_tcp((0, 0, 0, 0, 0, 0))
        gripper_controller = FingerGripperController(IP = self.IP, ur_connection = self.ur_connection)
        gripper_controller.connect_gripper()
        # robot.ur_connection.set_payload(2, (0, 0, 0.1))

        gripper_controller.pick(pos1)
        gripper_controller.place(pos2)
        print('Finished transfer')
        gripper_controller.disconnect_gripper()

    def run_urp_program(self, transfer_file_path:str = None, program_name: str = None):

        """"""
        if not program_name:
            print("Provide program name!")
            return
        
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
    robot = UR(IP="192.168.1.100")
    print(robot.get_joint_angles())
    # log = robot.run_urp_program(program_name="chemspeed2tecan.urp")
    # print(log)
    # robot.transfer
    # (robot.plate_exchange_1,robot.plate_exchange_1)
    # for i in range(1000):
    #     print(robot.get_movement_state())
    #     robot.get_overall_robot_status()
    #     sleep(0.5)

    robot.ur_connection.disconnect_ur()





