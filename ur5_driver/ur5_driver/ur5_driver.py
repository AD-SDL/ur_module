#!/usr/bin/env python3

import threading
import socket 
import epics

from multiprocessing.connection import wait
from time import sleep
from copy import deepcopy

from ur_dashboard import UR_DASHBOARD
import robotiq_gripper as robotiq_gripper
from urx import Robot, RobotException

class UR5(UR_DASHBOARD):
    

    def __init__(self, IP:str = "146.137.240.38", PORT: int = 29999, gripper:bool = False, tool_changer_pv:str = None, pipette_pv:str = None, camera_pv:str = None):
        
        super().__init__(IP=IP, PORT=PORT)

        # ur SETUP:
        self.ur = None
        self.gripper = None
        self.pipette = None
        self.tool_changer = None
        self.camera = None
    
        self.gripper_close = 130 # 0-255 (255 is closed)
        self.griper_open = 0
        self.gripper_speed = 150 # 0-255
        self.gripper_force = 0 # 0-255

        self.connect_ur()
        if gripper:
            self.connect_gripper()
        if tool_changer_pv:
            self.connect_tool_changer(tool_changer_pv)
        if pipette_pv:
            self.connect_pipette(pipette_pv)
        if camera_pv:
            self.connect_camera(camera_pv)

        self.pipette_drop_tip_value = -8
        self.pipette_aspirate_value = 2.0
        self.pipette_dispense_value = -2.0
        self.droplet_value = 0.3

        self.acceleration = 0.5
        self.velocity = 0.5
        self.robot_current_joint_angles = None

        self.get_movement_state()

        self.module_entry = [-0.1828145484680406, 0.1501917529215074, 0.4157045667286946, -0.014753354925067616, -3.133785224432585, -0.01020982277167234]
        self.module_entry_joint = [-1.3963525930987757, -2.1945158443846644, 2.1684568564044397, -1.5495260164937754, -1.5337546507464808, 3.2634336948394775]
        self.home = [-0.13358071546889347, -0.009673715752021885, 0.5890782758304143, -0.014566051910791617, -3.133734935087693, -0.010359747956377084]
        self.home_joint = [-1.355567757283346, -2.5413090191283167, 1.8447726408587855, -0.891581193809845, -1.5595606009112757, 3.3403327465057373]
        self.plate_exchange_1_above = [-0.18284724105645211, 0.7914820291585895, 0.41175512257988434, -0.014545475433050672, -3.1337759450718, -0.010278634391729295]
        self.plate_exchange_1 = [-0.1828537989205587, 0.7914917511283945, 0.390542100409092, -0.014571172649734884, -3.133719848650817, -0.010138239501312422]


    def connect_ur(self):
        """
        Description: Create conenction to the UR robot
        """

        for i in range(10):
            try:
                self.ur = Robot(self.IP)

            except socket.error:
                print("Trying robot connection ...")
                sleep(10)

            else:
                print('Successful ur connection')
                break

    def connect_gripper(self):
        """
        Connect to the gripper
        """
        try:
            # GRIPPER SETUP:
            self.gripper = robotiq_gripper.RobotiqGripper()
            print('Connecting to gripper...')
            self.gripper.connect(self.IP, 63352)

        except Exception as err:
            print("Gripper error: ", err)

        else:
            if self.gripper.is_active():
                print('Gripper already active')
            else:
                print('Activating gripper...')
                self.gripper.activate()
                print('Opening gripper...')
                self.gripper.move_and_wait_for_pos(self.griper_open, self.gripper_speed, self.gripper_force)

    def connect_tool_changer(self, tool_changer_pv:str):
        """
        Connect tool changer
        """

        try:
            # Establishing a connection with the tool changer using EPICS library.
            self.tool_changer = epics.PV(tool_changer_pv)

        except Exception as err:
            print("Tool changer error: ", err)

        else:
            print("Tool changer is connected.")

    def connect_pipette(self, pipette_pv:str):
        """
        Connect pipette
        """

        try:
            # Establishing a connection with the pipette using EPICS library.
            self.pipette = epics.PV(pipette_pv)

        except Exception as err:
            print("Pipette error: ", err)

        else:
            print("Pipette is connected.")

    def connect_camera(self, camera_pv:str):
        """
        Connect camera
        """

        try:
            # Establishing a connection with the camera using EPICS library.
            self.camera =  epics.PV("8idiARV1:cam1:Acquire")
            self.cam_image = epics.PV("8idiARV1:Pva1:Image")
            self.cam_capture =  epics.PV("8idiARV1:Pva1:Capture")

        except Exception as err:
            print("Pipette error: ", err)

        else:
            print("Pipette is connected.")

    def disconnect_ur(self):
        """
        Description: Disconnects the socket connection with the UR robot
        """
        self.ur.close()
        print("Robot connection is closed.")

    def get_joint_angles(self):
        
        return self.ur.getj()
    
    def get_cartesian_coordinates(self):
        
        return self.ur.getl()
    
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

    def home(self):
        """
        Description: Moves the robot to the home location.
        """
        print("Homing the robot...")
        self.ur.movej(self.home_joint, self.acceleration, self.velocity, 0, 0)
        sleep(4)

        print("Robot moved to home location")


    def pick(self, pick_goal):

        '''Pick up from first goal position'''

        above_goal = deepcopy(pick_goal)
        above_goal[2] += 0.05

        print('Moving to home position')
        # self.ur.movel(self.home, self.acceleration, self.velocity)
        self.ur.movej(self.home_joint, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.ur.movel(self.module_entry, self.acceleration, self.velocity)
        self.ur.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to above goal position')
        self.ur.movel(above_goal, self.acceleration, self.velocity)

        print('Moving to goal position')
        self.ur.movel(pick_goal, self.acceleration, self.velocity)

        print('Closing gripper')
        self.gripper.move_and_wait_for_pos(self.gripper_close, self.gripper_speed, self.gripper_force)

        print('Moving back to above goal position')
        self.ur.movel(above_goal, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.ur.movel(self.module_entry, self.acceleration, self.velocity)
        self.ur.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to home position')
        # self.ur.movel(self.home, self.acceleration, self.velocity)
        self.ur.movej(self.home_joint, self.acceleration, self.velocity)


    def place(self, place_goal):

        '''Place down at second goal position'''

        above_goal = deepcopy(place_goal)
        above_goal[2] += 0.05

        print('Moving to home position')
        # self.ur.movel(self.home, self.acceleration, self.velocity)
        self.ur.movej(self.home_joint, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.ur.movel(self.module_entry, self.acceleration, self.velocity)
        self.ur.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to above goal position')
        self.ur.movel(above_goal, self.acceleration, self.velocity)

        print('Moving to goal position')
        self.ur.movel(place_goal, self.acceleration, self.velocity)

        print('Opennig gripper')
        self.gripper.move_and_wait_for_pos(self.griper_open, self.gripper_speed, self.gripper_force)

        print('Moving back to above goal position')
        self.ur.movel(above_goal, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.ur.movel(self.module_entry, self.acceleration, self.velocity)
        self.ur.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to home position')
        # self.ur.movel(self.home, self.acceleration, self.velocity)
        self.ur.movej(self.home_joint, self.acceleration, self.velocity)

        
    def transfer(self, pos1, pos2, gripper_rotation:str = None, safe_heigh: int = None):
        ''''''
        self.ur.set_tcp((0, 0, 0, 0, 0, 0))
        # robot.ur.set_payload(2, (0, 0, 0.1))

        self.pick(pos1)
        self.place(pos2)
        print('Finished transfer')

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
        sleep(2)
        
        print("Running the URP program: ", program_name)
        time_elapsed = 0
        program_err = ""

        while "false" not in self.get_program_run_status():
            time_elapsed += 1 
            sleep(1)
            program_state = self.get_program_state()

            if "PAUSED" in program_state:
                program_err = self.get_safety_status()

        if "STOPPED" in program_state:       
            program_log = {"output_code":"0", "output_msg": "Successfully finished " + program_name, "output_log": "seconds_elapsed:" + str(time_elapsed)}
        elif "PAUSED" in program_state:
            program_log = {"output_code":"-1", "output_msg": "Failed running: " + program_name, "output_log": program_err}
        else:
            program_log = {"output_code":"-1", "output_msg": "Unkown program state:  " + program_name, "output_log": program_state}

        return program_log


if __name__ == "__main__":

    pos1= [-0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    pos2= [0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    
    robot = UR5("146.139.48.76", gripper = True)
    log = robot.run_urp_program(program_name="chemspeed2tecan.urp")
    print(log)
    # robot.transfer(robot.plate_exchange_1,robot.plate_exchange_1)
    # for i in range(1000):
    #     print(robot.get_movement_state())
    #     robot.get_overall_robot_status()
    #     sleep(0.5)

    robot.disconnect_ur()
