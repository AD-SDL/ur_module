from time import sleep
from copy import deepcopy

from .robotiq_gripper_driver import RobotiqGripper

class FingerGripperController():
    

    def __init__(self, hostname:str = "146.137.240.38", port: int = 63352, ur = None):
        """
        Constructor for the FingerGripperController class.

        Args:
            hostname (str): The hostname of the robot.
            port (int): Port number to connect to the robot over the Interpreter socket
        """
        self.host = hostname
        self.PORT = port
        
        if not ur:
            raise Exception("UR connection is not established")
        else:
            self.ur = ur
            self.robot = self.ur.ur_connection
            # self.robot.set_payload(1.2)# TODO: Check the actual payload

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
            self.gripper = RobotiqGripper()
            print('Connecting to gripper...')
            self.gripper.connect(hostname = self.host, port = self.PORT)
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

    def disconnect_gripper(self):
        """
        Discconect from the gripper
        """
        try:
            self.gripper.disconnect()
        except Exception as err:
            print("Gripper error: ", err)

        else:
            print("Gripper conenction is closed")
            
    def pick(self, pick_goal:list = None, approach_axis:str = "z", approach_distance:float = 0.05):

        '''Pick up from first goal position'''

        if not pick_goal:
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

        above_goal = deepcopy(pick_goal)
        above_goal[2] += 0.05

        print('Moving to home position')
        # self.robot.movel(self.home, self.acceleration, self.velocity)
        self.robot.movej(self.home_joint, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.robot.movel(self.module_entry, self.acceleration, self.velocity)
        self.robot.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to above goal position')
        self.robot.movel(above_goal, self.acceleration, self.velocity)

        print('Moving to goal position')
        self.robot.movel(pick_goal, self.acceleration, self.velocity)

        print('Closing gripper')
        self.gripper.move_and_wait_for_pos(self.gripper_close, self.gripper_speed, self.gripper_force)

        print('Moving back to above goal position')
        self.robot.movel(above_goal, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.robot.movel(self.module_entry, self.acceleration, self.velocity)
        self.robot.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to home position')
        # self.robot.movel(self.home, self.acceleration, self.velocity)
        self.robot.movej(self.home_joint, self.acceleration, self.velocity)

    def place(self, place_goal:list = None, approach_axis:str = "z", approach_distance:float = 0.05):

        '''Place down at second goal position'''
        if not place_goal:
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

        above_goal = deepcopy(place_goal)
        above_goal[axis] += 0.05

        print('Moving to above goal position')
        self.robot.movel(above_goal, self.acceleration, self.velocity)

        print('Moving to goal position')
        self.robot.movel(place_goal, self.acceleration, self.velocity)

        print('Opennig gripper')
        self.gripper.move_and_wait_for_pos(self.griper_open, self.gripper_speed, self.gripper_force)

        print('Moving back to above goal position')
        self.robot.movel(above_goal, self.acceleration, self.velocity)

 
class VacuumGripperController():
    

    def __init__(self, IP:str = "146.137.240.38", PORT: int = 29999, gripper:bool = False):
        
        super().__init__(IP=IP, PORT=PORT)
