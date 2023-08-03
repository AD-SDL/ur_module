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
        self.MOVE_ACC = 0.5
        self.MOVE_VEL = 0.5

        self.default_yolo_model_path = "best.pt"
        self.class_names = ['deepwellplates', 'tipboxes', 'hammers', 'wellplates', 'wellplate_lids'] 
        self.target_object = target_object.lower()
        self.model = None
        self.object_distance = 0 
        self.object_reference_frame = None

        self.image_size_x = 640
        self.image_size_y = 480

        if self.target_object not in self.class_names:
            raise Exception("Target object category doesn't exist in the trained model class list")
        
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
        depth_frame = frames.get_depth_frame()
        img = np.asanyarray(color_frame.get_data())
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (self.image_size_x, self.image_size_y))

        return img, color_frame, depth_frame
    
    def load_yolo_model(self, model_path = None):
        """
        """
        if not model_path:
            model_file_path = self.default_yolo_model_path
        else:
            model_file_path = model_path

        # Load the trained YOLO model
        self.model = YOLO(model_file_path)

    def get_object_reference_frame(self, depth_frame, center_x, center_y):
        # Get the intrinsic parameters of the depth frame
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        # Use the depth value and the center of the bounding box to get the 3D coordinates of the object
        self.object_reference_frame = realsense.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], self.object_distance)

    def move_to_object(self):
        """Method to move the robot arm to the object"""

        # Extract the x, y, and z coordinates
        trans_x = self.object_reference_frame[0]
        trans_y = self.object_reference_frame[1]
        trans_z = self.object_reference_frame[2]
        # Print out the object's 3D coordinates
        print("Object XYZ: " + str(self.object_reference_frame))

        if trans_z != 0:
            # Move the robot's tool (e.g. a gripper) to be centered over the object in the x-y plane
            self.ur_connection.translate_tool([-trans_x, -trans_y, 0], acc=self.MOVE_ACC, vel=self.MOVE_VEL)
        
    def center_the_gripper(self, object_center = None):
        """
        Method to center the robot gripper over the detected object in its field of view.
        
        Args:
            object_center: Optional; pre-calculated object center coordinates.

        Returns:
            None
        """

        # If no object_center is provided, the function terminates early
        if object_center is None:
            return

        # Capture image and get color and depth frames
        img, color_frame, depth_frame = self.capture_image()

        # If any of these frames or the image is not available, terminate the function
        if not color_frame or not depth_frame or not img:
            raise ValueError("Could not capture image or retrieve color/depth frames")

        boxes = self.model(img)[0].boxes  # Perform object detection

        for (xmin, ymin, xmax, ymax), cls in zip(boxes.xyxy, boxes.cls):
            self.object_distance = depth_frame.get_distance(int((xmin + xmax) / 2), int((ymin + ymax) / 2))
            center_x = int((xmin + xmax) / 2)
            center_y = int((ymin + ymax) / 2)

            cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
            cv2.putText(img, f"{self.object_distance:.2f}m", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.circle(img, (self.image_size_x/2, self.image_size_y/2), 5, (0, 0, 255), -1)

            self.get_object_reference_frame(depth_frame, center_x, center_y)
            self.move_to_object()
            break

