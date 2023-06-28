#!/usr/bin/env python3

import threading
import socket 
import epics

from multiprocessing.connection import wait
from time import sleep
from copy import deepcopy
import json

from ur_driver.ur_dashboard import UR_DASHBOARD
import ur_driver.robotiq_gripper as robotiq_gripper
from urx import Robot, RobotException

class Connection():
    """Connection to the UR robot to be shared within UR driver """
    def __init__(self,  IP:str = "146.137.240.38", PORT: int = 29999) -> None:
        self.IP = IP
        self.PORT = PORT
        self.connection = None

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


class UR(UR_DASHBOARD):
    

    def __init__(self, IP:str = "146.137.240.38", PORT: int = 29999, gripper:bool = False, tool_changer_pv:str = None, pipette_pv:str = None, camera_pv:str = None):
        
        super().__init__(IP=IP, PORT=PORT)

        self.ur = None
    
        self.connect_camera(camera_pv)

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

    def home(self, home_location = None):
        """
        Description: Moves the robot to the home location.
        """
        print("Homing the robot...")
        if home_location:
            home_loc = home_location
        else:
            home_loc = [-1.355567757283346, -2.5413090191283167, 1.8447726408587855, -0.891581193809845, -1.5595606009112757, 3.3403327465057373]
        self.ur.movej(home_loc, self.acceleration, self.velocity, 0, 0)
        sleep(4)

        print("Robot moved to home location")

    def pick_pipette(self):

        pipette_controller = PipetteController(ur_connection=self.ur)
        tool_changer_controller = ToolChangerController()

        pipette_controller.move_pipette_dock()
        tool_changer_controller.lock_tool_changer()
        pipette_controller.lift_pipette_on_dock()

    def place_pipette(self):

        pipette_controller = PipetteController(ur_connection=self.ur)
        tool_changer_controller = ToolChangerController()

        pipette_controller.move_pipette_dock()
        tool_changer_controller.unlock_tool_changer()
        pipette_controller.lift_pipette_on_dock()  

    def create_sample(self):
        """"""

        home_J = [2.017202138900757, -1.137721137409546, -0.9426093101501465, -2.6425615749754847, -4.693090263997213, -3.8424256483661097]

        pipette_controller = PipetteController(ur_connection=self.ur)
        self.home(home_J)
        pipette_controller.pick_tip()
        pipette_controller.make_sample()
        self.home(home_J)

    def run_droplet(self):
        pipette_controller = PipetteController(ur_connection=self.ur)
        pipette_controller.create_droplet()
        pipette_controller.retrieve_droplet()

    def dispose_tip(self):
        pipette_controller = PipetteController(ur_connection=self.ur)
        home_J = [2.017202138900757, -1.137721137409546, -0.9426093101501465, -2.6425615749754847, -4.693090263997213, -3.8424256483661097]

        self.home(home_J)
        pipette_controller.empty_tip()
        pipette_controller.drop_tip_to_trash()
        self.home(home_J)
        
    def droplet_exp(self, tip_number_1:int = None, tip_number_2:int = None):
        """
        Description: Runs the full droplet experiment by calling the functions that perform each step in the experiment.
        """
        print("-*-*-* Starting the droplet experiment *-*-*-")
        home_J = [2.017202138900757, -1.137721137409546, -0.9426093101501465, -2.6425615749754847, -4.693090263997213, -3.8424256483661097]

        pipette_controller = PipetteController(ur_connection=self.ur)
        tool_changer_controller = ToolChangerController()

 

        pipette_controller.move_pipette_dock()
        tool_changer_controller.lock_tool_changer()
        pipette_controller.lift_pipette_on_dock()    

        self.home(home_J)
        pipette_controller.empty_tip()
        pipette_controller.drop_tip_to_trash()
        self.home(home_J)

        pipette_controller.move_pipette_dock()
        tool_changer_controller.unlock_tool_changer()
        pipette_controller.lift_pipette_on_dock()         
        print("-*-*-* Droplet experiment is completed *-*-*-")
        
    def gripper_transfer(self, pos1, pos2, gripper_rotation:str = None, safe_heigh: int = None):
        '''
        Make a transfer using the finger gripper
        ''' 
        self.ur.set_tcp((0, 0, 0, 0, 0, 0))
        gripper_controller = GripperController(IP = self.IP, ur_connection = self.ur)
        # robot.ur.set_payload(2, (0, 0, 0.1))

        gripper_controller.pick(pos1)
        gripper_controller.place(pos2)
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
        sleep(5)
        
        print("Running the URP program: ", program_name)
        time_elapsed = 0
        program_err = ""
        
        program_status = "BUSY"
        ready_status_count = 0
        while program_status == "BUSY":
            if self.get_movement_state() == "READY":
                ready_status_count += 1
                if ready_status_count >=3:
                    program_status = "READY"
            else:
                ready_status_count = 0
            sleep(2)

        #TODO: FIX the output loggings 

        # if "STOPPED" in program_state:       
        program_log = {"output_code":"0", "output_msg": "Successfully finished " + program_name, "output_log": "seconds_elapsed:" + str(time_elapsed)}
        # elif "PAUSED" in program_state:
            # program_log = {"output_code":"-1", "output_msg": "Failed running: " + program_name, "output_log": program_err}
        # else:
            # program_log = {"output_code":"-1", "output_msg": "Unkown program state:  " + program_name, "output_log": program_state}

        return program_log


# if __name__ == "__main__":

#     pos1= [-0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
#     pos2= [0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    
#     robot = UR("146.139.48.76", gripper = True)
#     log = robot.run_urp_program(program_name="chemspeed2tecan.urp")
#     print(log)
#     # robot.transfer
#     # (robot.plate_exchange_1,robot.plate_exchange_1)
#     # for i in range(1000):
#     #     print(robot.get_movement_state())
#     #     robot.get_overall_robot_status()
#     #     sleep(0.5)

#     robot.disconnect_ur()


class GripperController():
    

    def __init__(self, IP:str = "146.137.240.38", ur_connection = None):
        self.IP = IP
        self.PORT = 63352
        self.ur = ur_connection

        self.gripper_close = 130 # 0-255 (255 is closed)
        self.griper_open = 0
        self.gripper_speed = 150 # 0-255
        self.gripper_force = 0 # 0-255

        self.acceleration = 0.5
        self.velocity = 0.2
        self.speed_ms    = 0.750
        self.speed_rads  = 0.750
        self.accel_mss   = 1.200
        self.accel_radss = 1.200
        self.blend_radius_m = 0.001
        self.ref_frame = [0,0,0,0,0,0]

        self.module_entry = [-0.1828145484680406, 0.1501917529215074, 0.4157045667286946, -0.014753354925067616, -3.133785224432585, -0.01020982277167234]
        self.module_entry_joint = [-1.3963525930987757, -2.1945158443846644, 2.1684568564044397, -1.5495260164937754, -1.5337546507464808, 3.2634336948394775]
        self.home = [-0.13358071546889347, -0.009673715752021885, 0.5890782758304143, -0.014566051910791617, -3.133734935087693, -0.010359747956377084]
        self.home_joint = [-1.355567757283346, -2.5413090191283167, 1.8447726408587855, -0.891581193809845, -1.5595606009112757, 3.3403327465057373]
        self.plate_exchange_1_above = [-0.18284724105645211, 0.7914820291585895, 0.41175512257988434, -0.014545475433050672, -3.1337759450718, -0.010278634391729295]
        self.plate_exchange_1 = [-0.1828537989205587, 0.7914917511283945, 0.390542100409092, -0.014571172649734884, -3.133719848650817, -0.010138239501312422]

        
    def connect_gripper(self):
        """
        Connect to the gripper
        """
        try:
            # GRIPPER SETUP:
            self.gripper = robotiq_gripper.RobotiqGripper()
            print('Connecting to gripper...')
            self.gripper.connect(self.IP, self.PORT)

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

class VacuumGripperController():
    

    def __init__(self, IP:str = "146.137.240.38", PORT: int = 29999, gripper:bool = False):
        
        super().__init__(IP=IP, PORT=PORT)

class ScrewdriverController():
    

    def __init__(self ):
        """"""
        self.home_urp = "home_screwdriver.urp"
        self.drive_forward_urp = "home_screwdriver.urp"
        self.driver_backward_urp = "home_screwdriver.urp"

    def check_screwdriver_controls(self):
        pass  
    def get_urp_programs(self):
        # Read from file and save it on the ur 
        pass
    def update_urp_programs(self):
        pass

class PipetteController():

    def __init__(self, pipette_pv:str = None, ur_connection = None):

        self.pv = pipette_pv
        self.ur = ur_connection

        self.acceleration = 0.5
        self.velocity = 0.2
        self.speed_ms    = 0.750
        self.speed_rads  = 0.750
        self.accel_mss   = 1.200
        self.accel_radss = 1.200
        self.blend_radius_m = 0.001
        self.ref_frame = [0,0,0,0,0,0]

        self.pipette_drop_tip_value = -8
        self.pipette_aspirate_value = 2.0
        self.pipette_dispense_value = -2.0
        self.droplet_value = 0.3

        self.pipette_loc = [-0.30710397664568057, 0.2223363316295067, 0.25346649921490616, 0.9780579194931717, -1.3456500374612195, -1.5122814896417478]
        self.pipette_loc_J = [2.8711442947387695, -1.8251310787596644, -1.5156354904174805, -1.3721376222423096, -4.720762554799215, -3.0886977354632776]
        self.pipette_approach = [-0.30710468347240427, 0.22234393663902577, 0.2793166289891617, 0.977973715529265, -1.3455795650125528, -1.512392593568845]
        self.pipette_approach_J = [2.8711442947387695, -1.8102451763548792, -1.412275791168213, -1.4902585309794922, -4.720990244542257, -3.088721577321188]
        self.pipette_above = [-0.3075719688934094, 0.2227307810713913, 0.40515454739546075, 0.9815940238325527, -1.3416684284127856, -1.504904936327573]
        self.pipette_above_J = [2.869802236557007, -1.9749981365599574, -0.5613865852355957, -2.1772977314391078, -4.720307175313131, -3.0981438795672815]
        self.tip1_loc = [0.049076405377552826, 0.35130426249264163, 0.063, 0.9759108742683295, -1.3350220046082053, -1.5092226077826993]
        self.tip1_approach = [0.049076095756735466, 0.3513032390285145, 0.083, 0.9758916159413838, -1.3350252553821587, -1.5092057412143818]
        self.tip1_above = [0.04908221782054774, 0.3513003341332178, 0.138, 0.9758574103691817, -1.3350463108315163, -1.5091909291569083]
        self.tip2_loc = [0.04909177440821851, 0.3411316353820866, 0.0628, 0.977119433532159, -1.3337829736507698, -1.5108373189678133]
        self.tip2_approach = [0.04909177440821851, 0.3411316353820866, 0.083, 0.977119433532159, -1.3337829736507698, -1.5108373189678133]
        self.tip2_above = [0.04909177440821851, 0.3411316353820866, 0.138, 0.977119433532159, -1.3337829736507698, -1.5108373189678133]
        self.sample1 = [0.15220619381604186, 0.21043816573205595, 0.09618091909170277, 1.444826407763332, -0.2548060433102738, -0.31289353129621067]
        self.sample1_above = [0.15220723461648447, 0.2104311001071656, 0.14402782259610025, 1.4448359749910735, -0.2548206714588542, -0.31295915781137074]
        self.sample2_above = [0.15279755520703087, 0.18939793717407497, 0.14402267332894347, 1.444821393022025, -0.25485812796155616, -0.3128929822914916]
        self.sample2 = [0.15186061464767017, 0.18822197623964088, 0.09490910394912143, 1.4440966224799245, -0.255613147568461, -0.3122426586441542]
        self.empty_tube = [0.15203368788019977, 0.16531582069324421, 0.12185568609417977, 1.4402850302548993, -0.2846256403901101, -0.3468228184833902]
        self.empty_tube_above = [0.15203001904780783, 0.16531236663764431, 0.14222620538915642, 1.4402337440190125, -0.2846450307479814, -0.346876615018759]
        self.well1 = [0.12772478460859046, 0.21370236710062357, 0.08390608100945282, 1.4380130231592743, -0.2414629895555231, -0.2954608172533908]
        self.well1_above = [0.12773445855037924, 0.21371308008717516, 0.1271232135439438, 1.4380596200664426, -0.24151536289689018, -0.2954919320386042]
        self.trash_bin_above = [0.187412530306272, 0.2868009561100828, 0.12712991727750073, 1.438076830279249, -0.2414934112798892, -0.2954944172453427]
        self.trash_bin = [0.1874179391982658, 0.2867862635600429, 0.013156853887081085, 1.438022625162957, -0.24148065729851562, -0.2954808450568972]

    def connect_pipette(self):
        """
        Connect pipette
        """

        try:
            # Establishing a connection with the pipette using EPICS library.
            self.pipette = epics.PV(self.pv)

        except Exception as err:
            print("Pipette error: ", err)

        else:
            print("Pipette is connected.")
    
    def move_pipette_dock(self):
        """
        Description: Moves the robot to the doscking location and then picks up the pipette.
        """
        print("Picking up the pipette...")
        accel_mss = 1.00
        speed_ms = 1.00
    
        print("Picking up the pipette...")
        sleep(1)
        self.ur.movel(self.pipette_above,self.accel_mss,speed_ms,0,0)
        sleep(2)
        self.ur.movel(self.pipette_approach,self.accel_mss,speed_ms,0,0)
        speed_ms = 0.01
        sleep(1)
        self.ur.movel(self.pipette_loc,self.accel_mss,speed_ms,0,0)
        sleep(5)
        # LOCK THE TOOL CHANGER TO ATTACH THE PIPETTE HERE

    def lift_pipette_on_dock(self):
        """
        Description: Moves the robot to the doscking location and then picks up the pipette.
        """
        sleep(5.0)
        self.ur.movel(self.pipette_approach,self.accel_mss,speed_ms,0,0)
        sleep(1)
        speed_ms = 0.1
        self.ur.movel(self.pipette_above,self.accel_mss,speed_ms,0,0)
        sleep(2)
        print("Pipette successfully picked up")
       

    def pick_tip(self, x=0, y=0):
        """
        Description: Picks up a new tip from the first location on the pipette bin.
        """

        print("Picking up the first pipette tip...")
        speed_ms = 0.100

        self.ur.movel(self.tip1_above,self.accel_radss,self.speed_rads,0,0)
        sleep(2)
        speed_ms = 0.01
        self.ur.movel(self.tip1_approach,self.accel_radss,self.speed_rads,0,0)
        sleep(2)    
        self.ur.movel(self.tip1_loc,self.accel_mss,speed_ms,0,0)
        sleep(3)
        self.ur.movel(self.tip1_approach,self.accel_mss,speed_ms,0,0)
        sleep(2)
        speed_ms = 0.1
        self.ur.movel(self.tip1_above,self.accel_mss,speed_ms,0,0)
        sleep(2)
        print("Pipette tip successfully picked up")

    def pick_tip2(self, x=0, y=0):
        """
        Description: Picks up a new tip from the second location on the pipette bin.
        """
        
        print("Picking up the second pipette tip...")
        speed_ms = 0.100
        self.ur.movel(self.tip2_above,self.accel_radss,self.speed_rads,0,0)
        sleep(2)
        speed_ms = 0.01
        self.ur.movel(self.tip2_approach,self.accel_radss,self.speed_rads,0,0)
        sleep(2)    
        self.ur.movel(self.tip2_loc,self.accel_mss,speed_ms,0,0)
        sleep(3)
        self.ur.movel(self.tip2_approach,self.accel_mss,speed_ms,0,0)
        sleep(2)
        speed_ms = 0.1
        self.ur.movel(self.tip2_above,self.accel_mss,speed_ms,0,0)
        sleep(2)    
        print("Second pipette tip successfully picked up")

    def make_sample(self):
        
        """
        Description: 
            - Makes a new sample on the 96 well plate.
            - Mixes to liquits in a single well and uses a new pipette tip for each liquid.
            - In order to mix the liquids together, pipette performs aspirate and dispense operation multiple times in the well that contains both the liquids.
        """
        print("Making a sample using two liquids...")
        
        # MOVE TO THE FIRT SAMPLE LOCATION
        speed_ms = 0.1
        self.ur.movel(self.sample1_above,self.accel_mss,self.speed_ms,0,0)
        sleep(2)
        self.ur.movel(self.sample1,self.accel_mss,speed_ms,0,0)
        sleep(2)

        # ASPIRATE FIRST SAMPLE
        self.aspirate_pipette()
        self.ur.movel(self.sample1_above,self.accel_mss,speed_ms,0,0)
        sleep(1)

        # MOVE TO THE 1ST WELL
        self.ur.movel(self.well1_above,self.accel_mss,speed_ms,0,0)
        sleep(1)
        self.ur.movel(self.well1,self.accel_mss,speed_ms,0,0)
        sleep(1)

        # DISPENSE FIRST SAMPLE INTO FIRST WELL
        self.dispense_pipette()
        self.ur.movel(self.well1_above,self.accel_mss,speed_ms,0,0)
        sleep(1)

        # Changing tip
        self.drop_tip_to_trash()
        self.pick_tip2()

        # MOVE TO THE SECON SAMPLE LOCATION
        self.ur.movel(self.sample2_above,self.accel_mss,self.speed_ms,0,0)
        sleep(3)
        self.ur.movel(self.sample2,self.accel_mss,speed_ms,0,0)
        sleep(2)

        # ASPIRATE SECOND SAMPLE
        self.aspirate_pipette()       
        self.ur.movel(self.sample2_above,self.accel_mss,speed_ms,0,0)
        sleep(1)

        # MOVE TO THE 1ST WELL
        self.ur.movel(self.well1_above,self.accel_mss,speed_ms,0,0)
        sleep(1)    
        self.ur.movel(self.well1,self.accel_mss,speed_ms,0,0)
        sleep(1)

        # DISPENSE SECOND SAMPLE INTO FIRST WELL
        self.dispense_pipette()

        # MIX SAMPLE
        for i in range(3):
            self.aspirate_pipette()
            self.dispense_pipette()

        # Aspirate all the liquid   
        self.aspirate_pipette()
        self.aspirate_pipette()
        self.ur.movel(self.well1_above,self.accel_mss,speed_ms,0,0)
        sleep(1)
        print("Sample is prepared")


    def aspirate_pipette(self):
        """
        Description: 
            - Drives pipette to aspirate liquid. 
            - Number of motor steps to aspirate liquid is stored in "self.pipette_aspirate_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Aspirating the sample...")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) + self.pipette_aspirate_value)
        sleep(1)

    def dispense_pipette(self):
        """
        Description: 
            - Drives pipette to dispense liquid. 
            - Number of motor steps to dispense liquid is stored in "self.pipette_dispense_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Dispensing sample")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value)+ self.pipette_dispense_value)
        sleep(1)

    def create_droplet(self):
        """
        Description: 
            - Drives pipette to create a droplet.
            - Number of motor steps to create a droplet is stored in "self.droplet_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Creating a droplet...")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) - self.droplet_value)
        sleep(10)
  

    def retrieve_droplet(self):
        """
        Description: 
            - Retrieves the droplet back into the pipette tip.
            - Number of motor steps to retrieve a droplet is stored in "self.droplet_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Retrieving droplet...")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) + self.droplet_value + 0.5)
        sleep(1)
    
    def drop_tip_to_trash(self):        
        """
        Description: Drops the pipette tip by driving the pipette all the way to the lowest point.
        """

        print("Droping tip to the trash bin...")
        # Move to the trash bin location
        self.ur.movel(self.trash_bin_above, self.accel_mss, self.speed_ms,0,0)
        sleep(2)
        self.ur.movel(self.trash_bin, self.accel_mss, self.speed_ms, 0, 0)
        sleep(2)
        self.eject_tip()
        sleep(1)
        self.ur.movel(self.trash_bin_above, self.accel_mss, self.speed_ms,0,0)
        sleep(2)

    def eject_tip(self):
        """
        Description: Ejects the pipette tip
        """
        print("Ejecting the tip")
        self.pipette.put(self.pipette_drop_tip_value)
        sleep(2)
        self.pipette.put(0)
        sleep(2)
  
    def empty_tip(self):
        """
        Description: Dispenses all the liquid inside pipette tip.
        """
        print("Empting tip...")
        speed_ms = 0.5  
        # Moving the robot to the empty tube location
        self.ur.movel(self.empty_tube_above,self.accel_mss,self.speed_ms,0,0)
        sleep(2)
        speed_ms = 0.1
        self.ur.movel(self.empty_tube,self.accel_mss,speed_ms,0,0)
        sleep(2)

        # Drive the pipette three times to dispense all the liquid inside the pipette tip.
        for i in range(3):
            self.dispense_pipette()
            sleep(1)

        self.ur.movel(self.empty_tube_above,self.accel_mss,speed_ms,0,0)
        sleep(1)
    


class ToolChangerController():
    

    def __init__(self, tool_changer_pv:str = None):
        self.pv = tool_changer_pv
        self.connect_tool_changer()
        
    def connect_tool_changer(self):
        """
        Connect tool changer
        """

        try:
            # Establishing a connection with the tool changer using EPICS library.
            self.tool_changer = epics.PV(self.pv)

        except Exception as err:
            print("Tool changer error: ", err)

        else:
            print("Tool changer is connected.")

    
    def get_tool_changer_status(self):
        """
        Description: 
            - Gets the tool changer current status. 
            - Tool changer is controlled by pyepics PV commands.
        """
        status = self.tool_changer.get()
        return status

    def lock_tool_changer(self):
        """
        Description: 
            - Locks the tool changer. 
            - Tool changer is controlled by pyepics PV commands.
        """
        try:
            print("Locking the tool changer...")
            self.tool_changer.put(1)
        except Exception as err:
            print("Error accured while locking the tool changer: ", err)

    def unlock_tool_changer(self):
        """
        Description: 
            - Unlocks the tool changer. 
            - Tool changer is controlled by pyepics PV commands.
        """
        try:
            print("Unlocking the tool changer...")
            self.tool_changer.put(0)
        except Exception as err:
            print("Error accured while unlocking the tool changer: ", err)
