from time import sleep
from copy import deepcopy

import cv2
import pyrealsense2 as realsense


from transforms3d import euler, quaternions
from math import degrees

import torch
from torchvision.transforms import functional as F
from ultralytics import YOLO
import numpy as np

from robotiq_gripper_driver import RobotiqGripper
from urx import Robot

class CameraController():


    def __init__(self, robot_IP, ur_connection = None, target_object = None) -> None:
        """This is a camera controller class created for the UR robots to utilize Intel Realsense D400 series cameres. 
           Functionalities:  - Object detection based on a pre-trained YOLO model with 5 different labwares.
                             - Plan a robot trajectory to pick up the objects based on distance and object reference frame imformation obtained from the camera.

        :param [ur_connection]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ur_connection]: [class urx.Robot](connection to the urx.Robot)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        if not ur_connection:
            raise Exception("ur connection is not provided!")
        if not target_object:
            raise Exception("Target object for YOLO model is not provided")
        
        self.pipeline = None
        self.ur_connection =  ur_connection 

        self.default_yolo_model_path = "best.pt"
        self.model_object_list = ['deepwellplates', 'tipboxes', 'hammers', 'wellplates', 'wellplate_lids'] 
        self.target_object = target_object.lower()

        if self.target_object not in self.model_object_list:
            raise Exception("Target object category doesn't exist in the trained model object list")
        
        self.gripper = RobotiqGripper()
        print('Connecting to gripper...')
        self.gripper.connect(robot_IP, 63352)
        self.gripper.activate()
        self.gripper.move_and_wait_for_pos(0, 150, 0)
    
    def start_camera_stream(self) -> None:
        """ Starts the Intel realsense camera pipeline 
        :param:  None
        :return: None
        :return: None
        """
        self.pipeline = realsense.pipeline()
        config = realsense.config()
        config.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30)  # Color stream configuration
        config.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30)  # Depth stream configuration
        self.pipeline.start(config)

    def capture_image(self) -> cv2:
        """ Capture a new image from the camera
        """
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        img = np.asanyarray(color_frame.get_data())
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img
    
    def load_yolo_model(self, model_path = None):
        """
        """
        if not model_path:
            model_file_path = self.default_yolo_model_path
        else:
            model_file_path = model_path

        # Load the trained YOLO model
        model = YOLO(model_file_path)
        # Set the desired objects to detect
        return model
