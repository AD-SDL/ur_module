#!/usr/bin/env python3

import threading
import socket 

from multiprocessing.connection import wait
from time import sleep
from copy import deepcopy
import json
from math import radians, degrees

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

class UR():
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
        
        # super().__init__(hostname=hostname, PORT=PORT)

        self.hostname = hostname
        self.PORT = PORT
        self.ur_dashboard = UR_DASHBOARD(hostname = self.hostname, PORT = self.PORT)
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
        self.ur_connection.movej(home_loc,2,2)
        sleep(3.5)

        print("Robot homed")
  
    def pick_tool(self, home, tool_loc, docking_axis = "y", payload = 0.12):
        """
            Picks up a tool using the given tool location
        """
        self.ur_connection.set_payload(payload)
        wingman_tool = WMToolChangerController(tool_location = tool_loc, docking_axis = docking_axis, ur = self.ur_connection)
        self.home(home)
        wingman_tool.pick_tool()
        self.home(home)    

    def place_tool(self, home, tool_loc, docking_axis = "y"):
        """
            Picks up a tool using the given tool location
        """
        wingman_tool = WMToolChangerController(tool_location = tool_loc, docking_axis = docking_axis, ur = self.ur_connection)
        self.home(home)
        wingman_tool.place_tool()
        self.home(home)  

    def gripper_transfer(self, home:list = None, source: list = None, target: list = None, source_approach_axis:str = None, target_approach_axis:str = None, source_approach_distance: float = None, target_approach_distance: float = None, gripper_open:int = None, gripper_close:int = None) -> None:
        '''
        Make a transfer using the finger gripper. This function uses linear motions to perform the pick and place movements.
        ''' 
        if not source or not target:
            raise Exception("Please provide both the source and target loactions to make a transfer")
        
        self.home(home)
        
        try:
            gripper_controller = FingerGripperController(hostname = self.hostname, ur = self.ur_connection)
            gripper_controller.connect_gripper()

            if gripper_open:
                gripper_controller.gripper_open = gripper_open
            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            gripper_controller.transfer(home = home, source = source, target = target,source_approach_axis = source_approach_axis, target_approach_axis = target_approach_axis, source_approach_distance = source_approach_distance, target_approach_distance = target_approach_distance)
            print('Finished transfer')
            gripper_controller.disconnect_gripper()

        except Exception as err:
            print(err)

        finally:
            gripper_controller.disconnect_gripper()
            self.home(home)

    def gripper_screw_transfer(self, home:list = None, target:list = None, screwdriver_loc: list = None, screw_loc: list = None, screw_time:float = 10, gripper_open:int = None, gripper_close:int = None) -> None:
        """
        Using custom made screwdriving solution.
        """

        self.home(home)

        try:
            gripper_controller = FingerGripperController(hostname = self.hostname, ur = self.ur_connection)
            gripper_controller.connect_gripper()

            if gripper_open:
                gripper_controller.gripper_open = gripper_open
            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            gripper_controller.pick(pick_goal = screwdriver_loc)

            # Pick screw
            self.home(home)
            above_goal = deepcopy(screw_loc)
            above_goal[2] += 0.06
            self.ur_connection.movel(above_goal, self.acceleration, self.velocity)
            self.ur_connection.movel(screw_loc, 0.2, 0.2)
            self.ur_connection.movel(above_goal, self.acceleration, self.velocity)

            # Move to the target location
            self.ur_connection.movel(target, self.acceleration, self.velocity)

            target_pose = [0,0,0.001,0,0,3.14] #Setting the screw drive motion
            self.ur_connection.speedl_tool(target_pose,2, screw_time) # This will perform screw driving motion for defined number of seconds
            print("Screwing down")
            sleep(screw_time+2)

            self.ur_connection.translate_tool([0,0,-0.03],0.5,0.5)
            self.home(home)

            gripper_controller.place(place_goal=hex_key)
            self.home(home)

        except Exception as err:
            print(err)

        finally:
            gripper_controller.disconnect_gripper()

    def gripper_unscrew(self, home:list = None, target:list = None, screwdriver_loc: list = None, screw_loc: list = None, screw_time:float = 10, gripper_open:int = None, gripper_close:int = None) -> None:
        """Perform unscrewing"""
        pass
    
    def remove_cap(self, target):
        pass
    
    def place_cap(self,target):
        pass
    
    def pick_and_flip_object(self, home:list = None, target: list = None, approach_axis:str = None, target_approach_distance: float = None, gripper_open:int = None, gripper_close:int = None) -> None:
        '''
        Pick an object then flips it and puts it back to the same location
        '''

        self.home(home)

        try:
            gripper_controller = FingerGripperController(hostname = self.hostname, ur = self.ur_connection)
            gripper_controller.connect_gripper()

            if gripper_open:
                gripper_controller.gripper_open = gripper_open
            if gripper_close:
                gripper_controller.gripper_close = gripper_close

            gripper_controller.pick(pick_goal = target, approach_axis = approach_axis)
    
            cur_j = self.ur_connection.getj()
            rotate_j = cur_j 
            rotate_j[5] += radians(180) 
            robot.ur_connection.movej(rotate_j,0.6,0.6)

            cur_l = self.ur_connection.getl()
            target[3] = cur_l[3]
            target[4] = cur_l[4]
            target[5] = cur_l[5]
            
            gripper_controller.place(place_goal = target, approach_axis = approach_axis)
            self.home(home)

        except Exception as er:
            print(er)
        finally:
            gripper_controller.disconnect_gripper()

    def robotiq_screwdriver_transfer(self, home:list = None, source: list = None, target: list = None, source_approach_axis:str = None, target_approach_axis:str = None, source_approach_distance: float = None, target_approach_distance: float = None) -> None:
        '''
        Make a screw transfer using the screwdriver. This function uses linear motions to perform the pick and place movements.
        ''' 

        self.home(home)

        try:
            sr = ScrewdriverController(hostname = self.hostname, ur = self.ur_connection, ur_dashboard = self.ur_dashboard)
            sr.screwdriver.activate_screwdriver()
            sr.transfer(source=source, target=target, source_approach_axis=source_approach_axis, target_approach_axis = target_approach_axis, source_approach_dist=source_approach_distance, target_approach_dist=target_approach_distance)
            sr.screwdriver.disconnect()
        except Exception as err:
            print(err)
        
        self.home(home)

    def pipette_transfer(self, home:list = None,  tip_loc: list = None, sample_loc:list = None, well_loc:list = None) -> None:
        '''
        Make a liquid transfer using the pipette. This function uses linear motions to perform the pick and place movements.
        ''' 
        if not tip_loc or not sample_loc:
            raise Exception("Please provide both the source and target loactions to make a transfer")
        self.home(home)
        
        try:
            pipette = TricontinentPipetteController(hostname = self.hostname, ur = self.ur_connection)
            pipette.connect_pipette()
            pipette.pick_tip(tip_loc=tip_loc)
            self.home(home)
            pipette.transfer_sample(sample_loc=sample_loc, well_loc=well_loc)
        except Exception as err:
            print(err)
        finally:
            pipette.disconnect_pipette()
            self.home(home)

        # TODO: Handle these steps better. Tread each action as another transfer 
   
    def run_droplet(self, home, tip_loc, sample_loc, droplet_loc, tip_trash):
        """Create droplet"""

        pipette = OTPipetteController(ur_connection = self.ur_connection, IP = self.hostname)
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

        """Transfers the urp programs onto the polyscope and initiates them"""
        if not program_name:
            raise ValueError("Provide program name!")
        
        ur_program_path = "/programs/" + program_name 

        if transfer_file_path:
            self.ur_dashboard.transfer_program(local_path = transfer_file_path, ur_path = ur_program_path)
            sleep(2)

        self.ur_dashboard.load_program(program_path = ur_program_path)
        sleep(2)
        self.ur_dashboard.run_program()
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

        program_log = {"output_code":"0", "output_msg": "Successfully finished " + program_name, "output_log": "seconds_elapsed:" + str(time_elapsed)}

        return program_log
    
if __name__ == "__main__":

    pos1= [-0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    pos2= [0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    robot = UR(hostname="164.54.116.129")
    # tool_loc = [0.32704628917562345, -0.1017379678362813, 0.3642503117806354, -2.1526354130031917, 2.2615882459741723, -0.04632031979240964]
    home = [0.5431541204452515, -1.693524023095602, -0.7301170229911804, -2.2898713550963343, 1.567720651626587, -1.0230830351458948]
    tip1 = [0.04639965460538513, 0.4292986855073111, 0.0924689410052111, -3.1413810571577048, 0.014647332926328135, 0.004028900798665303]
    sample = [0.07220331720579347, 0.21138438053671288, 0.11898933185468973, -3.141349185677643, 0.014592306794949944, 0.004077757329820521]
    droplet = [-0.21435167102697, 0.31117471247776396, 0.273829131948966, 3.126800328499299, -0.017429873171790906, -0.007516422536326644]
    # Tools
    tip_eject = [0.10270290312926324, 0.2862487614384484, 0.10155903555930355, 3.1268372121718597, -0.01760209112687455, -0.007607510036297549]
    pipette_loc = [0.21285670041158733, 0.1548897634390196, 0.005543999069077835, 3.137978068966478, -0.009313836267512065, -0.0008972976992386885]
    handE_loc = [0.3131286590368134, 0.15480163498252172, 0.005543999069077835, 3.137978068966478, -0.009313836267512065, -0.0008972976992386885]
    screwdriver_loc = [0.43804370307762014, 0.15513117190281586, 0.006677533813616729, 3.137978068966478, -0.009313836267512065, -0.0008972976992386885]
    
    target = [0.24769823122656057, -0.3389885625301465, 0.368077779916273, 2.1730827596713733, -2.264911265531878, 0.0035892213555669857]
    cell_screw = [0.28783440601230226, -0.28678518269403513, 0.3176342990205253, 3.1381071683977293, -0.009392392291173335, -0.0008605239429651419]
    # screw_holder = [0.21876722334540147, -0.27273358502932915, 0.39525473397805677, 3.0390618278038524, -0.7398330220514875, 0.016498425988567388]
    hex_key = [0.40061621427107863, -0.19851389684726614, 0.2195475541919895, 3.1374987322951102, -0.009368331063787221, -0.0007768712432287358]
    cell_holder = [0.43785674873555014, -0.1363043381282072, 0.21998506102422555, 3.1380513355558466, -0.009323037734842953, -0.0006690858747472434]
    assembly_deck = [0.3174903285108201, -0.08258211007606345, 0.11525282484663647, 1.2274734115134542, 1.190534780943193, -1.1813375188608897]
    assembly_above = [0.3184636928538083, -0.28653275588144745, 0.3479834847161999, 3.138072684284994, -0.009498947342442873, -0.0007708886741400893]
    gripper_close = 85
    # robot.home(home)


    # robot.pick_tool(home, pipette_loc,payload=1.2)

    
    # SCREWDRIVING ---------------------------
    # robot.pick_tool(home, screwdriver_loc,payload=3)
    # robot.screwdriver_transfer(home=home,source=hex_key,target=hex_key, source_approach_distance=0.04)
    # robot.place_tool(home,screwdriver_loc)
    #-----------------------------------------
     
    # GRIPPER CELL PICK & PLACE 
    # robot.pick_tool(home, handE_loc,payload=1.2)
    # robot.gripper_transfer(source = assembly_deck, target = assembly_deck, source_approach_axis="y", target_approach_axis="y", gripper_open = 190, gripper_close = 240)
    # robot.gripper_transfer(home = home, source = hex_key, target = hex_key, source_approach_axis="z", target_approach_axis="z", gripper_open = 120, gripper_close = 200)
    # robot.gripper_transfer(home = home, source = cell_holder, target = assembly_deck, source_approach_axis="z", target_approach_axis="y", gripper_open = 190, gripper_close = 240)
    # robot.gripper_transfer(home = home, source = assembly_deck, target = cell_holder, source_approach_axis="y", target_approach_axis="z", gripper_open = 190, gripper_close = 240)
    # robot.custom_screwdriver_transfer(home=home,screwdriver_loc=hex_key,screw_loc=cell_screw,target_above=assembly_above,target=assembly_above,gripper_open=120,gripper_close=200)
    # robot.place_tool(home, handE_loc)
    #-----------------------------------------
    # robot.pick_and_flip_object(home=home,target=assembly_deck,approach_axis="y",gripper_open=190,gripper_close=240)
    

    # ----------------------------------------
    # CELL ASSEMBLY
    # robot.pick_tool(home, handE_loc,payload=1.2)
    # robot.gripper_transfer(home = home, source = cell_holder, target = assembly_deck, source_approach_axis="z", target_approach_axis="y", gripper_open = 190, gripper_close = 240)
    # robot.gripper_screw_transfer(home=home,screwdriver_loc=hex_key,screw_loc=cell_screw,target=assembly_above,gripper_open=120,gripper_close=200)
    # robot.pick_and_flip_object(home=home,target=assembly_deck,approach_axis="y",gripper_open=190,gripper_close=240)
    # robot.place_tool(home,tool_loc=handE_loc)
    test_loc = [0.30364466226740844, -0.1243275644148994, 0.2844145579322907, 3.1380384242791366, -0.009336265404641286, -0.0007377624513656736]

    # # ADD PIPETTE HERE  
    # robot.pick_tool(home,tool_loc=pipette_loc,payload=1.2)
    robot.pipette_transfer(home=home,tip_loc=test_loc,sample_loc=test_loc,well_loc=test_loc)
    # robot.place_tool(home,tool_loc=pipette_loc)
    # robot.pick_tool(home, handE_loc,payload=1.2)
    # robot.custom_screwdriver_transfer(home=home,screwdriver_loc=hex_key,screw_loc=cell_screw,target_above=assembly_above,target=assembly_above,gripper_open=120,gripper_close=200)
    # robot.gripper_transfer(home = home, source = assembly_deck, target = cell_holder, source_approach_axis="y", target_approach_axis="z", gripper_open = 190, gripper_close = 240)
    # robot.place_tool(home, handE_loc)
    # robot.ur_connection.speedl_tool([0,0,0.001,0,0,3.14],2,15.3)
    robot.ur.disconnect_ur()





