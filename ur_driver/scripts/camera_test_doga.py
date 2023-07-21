
import cv2
import pyrealsense2 as rs
import time
import torch
import numpy as np
from torchvision.transforms import functional as F
from ultralytics import YOLO
import math
from math import degrees
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper as gripper
from urx import Robot

# Start the URX robot connection
robot = Robot("192.168.1.102")
home = [0.29276956938468857, 0.4911629986137578, 0.2089015639442738, 2.653589770443803, 0.6925181364927394, 0.994258374012048]
robot.movel(home, 0.2,0.2)

model_file_path = 'best.pt'
# Load the trained YOLO model
model = YOLO(model_file_path)
# Set the desired objects to detect
desired_objects = ['tipboxes'] #, 'tipboxes', 'hammers', 'deepwellplates', 'wellplate_lids']  #list of known objects
# Initialize the Intel RealSense camera pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # Color stream configuration
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream configuration
profile = pipeline.start(config) 

# def calculate_orientation_difference(pose1, pose2):
#     rpy1 = pose1.get()
#     rpy2 = pose2.()
#     rpy_difference = robot.(pose1[0], pose1[1], pose1[2], rpy1[0], rpy1[1], rpy1[2], pose2[0], pose2[1], pose2[2], rpy2[0], rpy2[1], rpy2[2])
#     return rpy_difference
# Wait for the next set of frames
# Start capturing and processing frames
# while True:
def allign_object():
        # # Wait for the next set of frames
    frames = pipeline.wait_for_frames()

    # Get color and depth frames
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        return

    # Convert the color frame to a numpy array
    img = np.asanyarray(color_frame.get_data())

    # Convert the frame to the format expected by the model
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # Convert to BGR format
    img = cv2.resize(img, (640, 480))
    center = (320,240)

    # rotation_matrix = cv2.getRotationMatrix2D(center, 45, 1.0)
    # height = 640
    # width = 480

    # # Calculate the new image dimensions after rotation
    # cos = np.abs(rotation_matrix[0, 0])
    # sin = np.abs(rotation_matrix[0, 1])
    # new_width = int((height * sin) + (width * cos))
    # new_height = int((height * cos) + (width * sin))

    # Adjust the rotation matrix for image padding
    # rotation_matrix[0, 2] += (new_width / 2) - center[0]
    # rotation_matrix[1, 2] += (new_height / 2) - center[1]

    # Apply the rotation to the image
    # rotated_img = cv2.warpAffine(img, rotation_matrix, (new_width, new_height))

    # Perform object detection on the frame
    boxes = model(img)[0].boxes  # Perform object detection
    # find_angle = model(rotated_img, conf=0.01)[0].boxes  # Perform object detection

    from pprint import pprint

    print(boxes.xyxy)
    print(boxes.cls)

    for (xmin, ymin, xmax, ymax), cls in zip(boxes.xyxy, boxes.cls):
        
        depth_value = depth_frame.get_distance(int((xmin + xmax) / 2), int((ymin + ymax) / 2))
        distance = depth_value
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2) #calculate the center of the object
        offset_object_x = 320 - center_x
        offset_object_y = 240 - center_y

        print("Offset from center (X, Y):", offset_object_x, offset_object_y)

        # Adjust the camera to center the object in the frame
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        object_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], depth_value)
        # cv2.imshow("Original Image", img)
        # split the string into indv. integers and store them into a list
        trans_x = object_point[0] 
        trans_y = object_point[1] 
        trans_z = object_point[2]
        print("XYZ: ", object_point)

        # if abs(object_point[0]) < 0.40 or abs(object_point[1]) < 0.40:
    
        # boxes = model(img)[0].boxes  # Perform object detection
        
        # Obtain the adjacent point above the object
        time.sleep(10)

        cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
        cv2.putText(img, f"{distance:.2f}m", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line
        cv2.circle(img, (320, 240), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line
        if abs(offset_object_x) < 10 and abs(offset_object_y) < 10:
            # robot.translate_tool([-trans_x, -trans_y, adjacent_length + 0.1], acc=0.4, vel=0.2)
            time.sleep(5)
            pass
        if trans_z != 0:
            robot.translate_tool([ -trans_x, -trans_y, 0], acc= 1, vel=0.2)

            

        # Stop the pipeline and release resources
        pipeline.stop()
        cv2.destroyAllWindows()
        return object_point
    
def move_onto_object(object_point):
            
        trans_z = object_point[2]
        
        # time.sleep(1)

        current_location_l = robot.getl()

        angle = robot.getl()[3]
        gripper_flat = current_location_l
        gripper_flat[3] = 3.14
        gripper_flat[4] = 0.0
        gripper_flat[5] = 0.0  
        # num_z = int(trans_z)

        # Obtain the adjacent point above the object
        adjacent_length = math.cos(degrees(angle)) * trans_z

        # time.sleep(1)
        robot.movel(gripper_flat, acc = 1 , vel = 0.2)

        
        # Print the adjacent length
        print("Adjacent Length: ", adjacent_length)

        # go towards adjacent point
        # time.sleep(0.5)

        robot.translate_tool([adjacent_length + 0.09, 0, 0 ], acc= 1, vel=0.2)
        time.sleep(10) 
def test():
            object_frame = robot.get_pose()
            
            gripper_roll = math.pi
            gripper_pitch = 0.0
            gripper_yaw = 0.0
            
            orientation_difference = calculate_orientation_difference(object_frame, [0, 0, 0, gripper_roll, gripper_pitch, gripper_yaw])

            robot.movel_tool([0,0,0, orientation_difference[0], orientation_difference[1], orientation_difference[2]], acc=0.5, vel=0.2)
    # for (xmin, ymin, xmax, ymax), cls in zip(find_angle.xyxy, find_angle.cls):
           
    #         center_x = int((xmin + xmax) / 2)
    #         center_y = int((ymin + ymax) / 2) #calculate the center of the object
    #         offset_object_x = 320 - center_x
    #         offset_object_y = 240 - center_y

    #         cv2.rectangle(rotated_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
    #         cv2.circle(rotated_img, (center_x, center_y), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line
    #         cv2.circle(rotated_img, (320, 240), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line

    # Display the color image with bounding find_angle
    
    #Create a new canvas with the same dimensions as the original image
    # canvas = np.zeros_like(img)
    # canvas[:img.shape[0], :img.shape[1]] = img

    # # Paste the rotated image onto the canvas
    # target_region = canvas[y_start:y_end, x_start:x_end]
    # resized_rotated_image = cv2.resize(rotated_image, (target_region.shape[1], target_region.shape[0]))

    # # Paste the resized rotated image onto the canvas
    # canvas[y_start:y_end, x_start:x_end] = resized_rotated_image

    # Apply the rotation to the image
    # rotated_image = cv2.warpAffine(img, rotation_matrix, (640, 480))

    # cv2.imshow("Object Detection", rotated_image)
    # cv2.imshow("Original Image", img)
    # cv2.imshow("Rotated Image", rotated_img)

    # cv2.imshow("Rotated Image", rotated_image)
    # cv2.imshow("Composite Canvas", canvas)

    # Exit the loop if 'q' is pressed


def main():
    object_point = allign_object()
    move_onto_robot(object_point)
    allign_object()
"""
This updated code creates a new canvas (initialized with zeros) with the same dimensions as the original image. The rotated image is then pasted onto the canvas, considering its position within the canvas based on the center point.

The resulting composite canvas will show the non-rotated image with the rotated image positioned correctly within it. You can modify the canvas if you prefer a different color or transparency for the padded area.

Don't forget to replace "path_to_your_image.jpg" with the actual path to your image file."""