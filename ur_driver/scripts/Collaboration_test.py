import cv2
import pyrealsense2 as rs
import time
import torch
import numpy as np
from torchvision.transforms import functional as F
from ultralytics import YOLO
from urx import Robot
from math import radians
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper


# get_joint_positions = 

# UR robot configuration
robot_ip = "192.168.1.102"
robot = Robot(robot_ip)
time.sleep(0.2)

# Load the YOLO model
model_file_path = '/home/rpl/Documents/best.pt'
model = YOLO(model_file_path)

# Set the desired objects to detect
desired_objects = ['wellplates'], #'tipboxes', 'hammers', 'deepwellplates', 'wellplate_lids']

# Initialize the Intel RealSense camera pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # Color stream configuration
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream configuration
profile = pipeline.start(config)

# Start capturing and processing frames
while True:
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
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    img = cv2.resize(img, (640, 480))
    
    # Perform object detection on the frame
    boxes = model(img)[0].boxes

    for (xmin, ymin, xmax, ymax), cls in zip(boxes.xyxy, boxes.cls):
        depth_value = depth_frame.get_distance(int((xmin + xmax) / 2), int((ymin + ymax) / 2))
        distance = depth_value
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2)
        offset_object_x = 320 - center_x
        offset_object_y = 240 - center_y

        print("Offset from center (X, Y):", offset_object_x, offset_object_y)

        cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
        cv2.putText(img, f"{distance:.2f}m", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.circle(img, (320, 240), 5, (0, 0, 255), -1)


    
    
    # # ************************Modifications 7/13************************
    def center_object_frame(offset_object_x, offset_object_y):
    # Moves the robot arm to center the object in the frame
        if offset_object_x > 0:

            pos = offset_object_x 
            while offset_object_x > 0:
                pos -= .1
                robot.movel(pos, 0, 0, 0, 0, 0, 0, 0)

        if offset_object_x < 0:
            pos = offset_object_x 
            while offset_object_x < 0:
                pos += .1
                robot.movel(pos, 0, 0, 0, 0, 0, 0, 0) 
        if offset_object_x > 0:

            p = offset_object_y 
            while offset_object_y > 0:
                p -= .1
                robot.movel(p, 0, 0, 0, 0, 0, 0, 0)

        if offset_object_y < 0:
            p = offset_object_y 
            while offset_object_y < 0:
                p += .1
                robot.movel(0, p, 0, 0, 0, 0, 0, 0) 
    center_object_frame()

    def calc_d2():
        get_actual_tcp_pose()
    
    # # Move the the gripper above the object
    # robot.movel(object_x_gripper, object_y_gripper, object_z_gripper + .1, 0, 0, accel= 0.4, vel = 0.2 )

    # # Potentially reposition the camera to center the object in the frame
    # # Make sure that the object aligns with the gripper before lowering arm

    # # Lower the arm to obtain the object
    # time.sleep(1)
    # robot.movel(object_x_gripper, object_y_gripper, object_z_gripper, 0, 0, accel= 0.4, vel = 0.2 )

    # grip = Robotiq_Two_Finger_Gripper()

    
    # if robot.get_joint_position()
    

    #*******************************************************************

    # Display the color image with bounding boxes
    cv2.imshow("Object Detection", img)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the pipeline and release resources
pipeline.stop()
cv2.destroyAllWindows()
