
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
def connect_robot():
    robot = Robot("192.168.1.102")
    home = [0.29276956938468857, 0.4911629986137578, 0.2089015639442738, 2.653589770443803, 0.6925181364927394, 0.994258374012048]
    robot.movel(home, acc=0.2, vel=0.2)
    return robot

def load_model():
    model_file_path = 'best.pt'
    # Load the trained YOLO model
    model = YOLO(model_file_path)
    # Set the desired objects to detect
    desired_objects = ['tipboxes'] #, 'tipboxes', 'hammers', 'deepwellplates', 'wellplate_lids']  #list of known objects
    return model

def start_streaming():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # Color stream configuration
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream configuration
    profile = pipeline.start(config)
    return pipeline

def get_object_center(boxes):
    if len(boxes) > 0:
        xmin, ymin, xmax, ymax = boxes[0].xyxy[0]
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2)
        return center_x, center_y
    else:
        return None

def allign_object(pipeline, model):
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None

        img = np.asanyarray(color_frame.get_data())
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (640, 480))

        boxes = model(img)[0].boxes  # Perform object detection
        object_center = get_object_center(boxes)
        break

    return object_center

def center_the_gripper(robot, model, object_center, pipeline):
    if object_center is None:
        return

    # Get color and depth frames again to compute object 3D coordinates
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        return

    img = np.asanyarray(color_frame.get_data())
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    img = cv2.resize(img, (640, 480))

    boxes = model(img)[0].boxes  # Perform object detection

    for (xmin, ymin, xmax, ymax), cls in zip(boxes.xyxy, boxes.cls):
        depth_value = depth_frame.get_distance(int((xmin + xmax) / 2), int((ymin + ymax) / 2))
        distance = depth_value
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2)

        cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
        cv2.putText(img, f"{distance:.2f}m", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.circle(img, (320, 240), 5, (0, 0, 255), -1)

        # Obtain the x, y, and z coordinates of the center of the object
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        object_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], depth_value)

        trans_x = object_point[0]
        trans_y = object_point[1]
        trans_z = object_point[2]

        # robot.translate_tool([-trans_x, -trans_y, 0], acc=0.2, vel=0.2)

        print("XYZ: " + str(object_point))

        if trans_z != 0:
            robot.translate_tool([-trans_x, -trans_y, 0], acc=1, vel=0.2)

        time.sleep(5)
        cv2.destroyAllWindows()

        return object_point

def move_over_object(robot, object_point):
    # Assuming the gripper's current location is above the object,
    # move down by 10 centimeters along the y-axis
    object_x, object_y, object_z = object_point
    desired_height = object_y - 0.1  # Move down 10 centimeters above the object
    robot.translate_tool([object_x, desired_height, object_z], acc=1, vel=0.2)

def gripper_position():
    # Points the gripper downwards to prepare the gripper to pick up object

    pos_x = 3.1415
    pos_z = 0


def adjust_gripper():
    # rotate the gripper so it's aligned with the object
    pass

def grab():
    pass

def main(): 
    robot = connect_robot()
    model = load_model()
    pipeline = start_streaming()

    object_center = allign_object(pipeline, model)

    while True:
        object_center = allign_object(pipeline, model)
        if object_center:
            object_point = center_the_gripper(robot, model, object_center, pipeline)
            # move_over_object(robot, object_point)
            break
    pipeline.close()

    move_over_object(robot, object_point)

if __name__ == "__main__":
    main()  
# # This updated code creates a new canvas (initialized with zeros) with the same dimensions as the original image. The rotated image is then pasted onto the canvas, considering its position within the canvas based on the center point.

# The resulting composite canvas will show the non-rotated image with the rotated image positioned correctly within it. You can modify the canvas if you prefer a different color or transparency for the padded area.

# Don't forget to replace "path_to_your_image.jpg" with the actual path to your image file."""