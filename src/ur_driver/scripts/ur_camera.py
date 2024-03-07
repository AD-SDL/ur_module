from math import cos, degrees
import time

import numpy as np

import cv2
import pyrealsense2 as rs

from torchvision.transforms import functional as F
import torch
from ultralytics import YOLO

from urx import Robot

class URCamera():
    def __init__(self ,ur_connection = None) -> None:

        self.robot = ur_connection
    
    def start_camera(self):
        # Define the file path for the YOLO model
        model_file_path = '/home/rpl/Documents/best.pt'

        # Load the trained YOLO model
        model = YOLO(model_file_path)

        # Set the desired objects to detect
        desired_objects = ['wellplates']#, 'tipboxes', 'hammers', 'deepwellplates', 'wellplate_lids']

        # Initialize the Intel RealSense camera pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # Color stream configuration
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream configuration
        profile = pipeline.start(config)
    

    def start_object_scan(self):
        # Wait for the next set of frames
        frames = pipeline.wait_for_frames()

        # Get color and depth frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert the color frame to a numpy array
        img = np.asanyarray(color_frame.get_data())

        # Convert the frame to the format expected by the model
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # Convert to BGR format
        img = cv2.resize(img, (640, 480))
        center = (320, 240)

    def detect_objects(self):
        # Perform object detection on the frame
        boxes = model(img)[0].boxes  # Perform object detection      
        # Draw the bounding box and distance on the image
        cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
        cv2.putText(img, f"{distance:.2f}m", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.circle(img, (320, 240), 5, (0, 0, 255), -1)

        # Iterate over detected objects
        for (xmin, ymin, xmax, ymax), cls in zip(boxes.xyxy, boxes.cls):
            depth_value = depth_frame.get_distance(int((xmin + xmax) / 2), int((ymin + ymax) / 2))
            distance = depth_value
            center_x = int((xmin + xmax) / 2)
            center_y = int((ymin + ymax) / 2)
  
    def calculate_object_distace(self):
            # obtain the x y and z distance of the center of the object to the center of the frame
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            object_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], depth_value)

    def rotate_image(self):
        # Define the initial rotation angle and increment
        current_angle = self.robot.getl([5])
        angle_increment = 10

        # Load the image
        img = cv2.imread("your_image.jpg")

        # Calculate the center of the image
        height, width = img.shape[:2]
        center = (width // 2, height // 2)

        # Initialize variables to track the smallest area and the corresponding best angle
        smallest_area = float('inf')
        best_angle = current_angle

        while current_angle <= 270:
            # Calculate the rotation matrix
            rotation_matrix = cv2.getRotationMatrix2D(center, current_angle, 1.0)

            # Calculate the new image dimensions after rotation
            cos = np.abs(rotation_matrix[0, 0])
            sin = np.abs(rotation_matrix[0, 1])
            new_width = int((height * sin) + (width * cos))
            new_height = int((height * cos) + (width * sin))

            # Adjust the rotation matrix for image padding
            rotation_matrix[0, 2] += (new_width / 2) - center[0]
            rotation_matrix[1, 2] += (new_height / 2) - center[1]

            # Apply the rotation to the image
            rotated_img = cv2.warpAffine(img, rotation_matrix, (new_width, new_height))

            # Perform object detection on the rotated image
            boxes = model(rotated_img)[0].boxes

            # Calculate the area of the bounding box
            xmin, ymin, xmax, ymax = boxes.xyxy[0]  # Assuming there is only one bounding box
            area = (xmax - xmin) * (ymax - ymin)

            # Check if the current area is smaller than the smallest area
            if area < smallest_area:
                smallest_area = area
                best_angle = current_angle

            # Break the loop if the area is as small as possible
            if smallest_area == 0:
                break

            current_angle += angle_increment

        # Display the best angle and smallest area
        print("Best Angle:", best_angle)
        print("Smallest Area:", smallest_area)

    def align_gripper(self):
        #before aligning the gripper with the object, the camera has to be facing downwards
        if best_angle != math.pi/2
            self.robot.movej([0, 0, 0, 0, ((math.pi/2) - angle), 0], acc=0.2, vel=0.2)            
        # rotate the gripper along the z axis to grab the image
        self.robot.movej([0, 0, 0, 0, 0, math.radians(best_angle)], acc = 0.2, vel = 0.2)    

    def move_object_top(self):
        # Adjust the camera to center the object in the frame
        self.robot.translate_tool([-object_point[0], -object_point[1], 0], acc=0.2, vel=0.2)
        print("XYZ: " + object_point)        

if __name__ == "__main__":
    r = Robot("192.168.1.100")
  
    ur_camera = URCamera(ur_connection=r)
    ur_camera.start_camera()
    ur_camera.start_object_scan()

    