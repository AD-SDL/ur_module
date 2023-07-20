
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
    # robot.movel(home, acc=0.2, vel=0.2)
    return robot

def load_model():
    model_file_path = '/home/rpl/wei_ws/src/ur_module/ur_driver/scripts/best.pt'
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

def capture_image(pipeline):
    # Capture a new image from the camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    img = np.asanyarray(color_frame.get_data())
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    return img

def get_object_center(boxes):
    if len(boxes) > 0:
        xmin, ymin, xmax, ymax = boxes[0].xyxy[0]
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2)
        return center_x, center_y
    else:
        return None

def allign_object(pipeline, model):
    object_center = None
    while object_center is None:
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
            break  # break after moving the robot over the first object

        # time.sleep(5)             
        # cv2.destroyAllWindows()

    return object_point

def move_over_object(object_point, robot):
    adjacent_length = get_adjacent_lenght(object_point,robot)

    current_location = robot.getl()
    gripper_flat = current_location
    gripper_flat[3] = 3.14
    gripper_flat[4] = 0.0
    gripper_flat[5] = 0.0

    robot.movel(gripper_flat, acc = 0.5, vel = 0.2)

    fixed_height = 0.00 #0.05
    desired_position = adjacent_length - fixed_height
    robot.translate_tool([0, desired_position, 0], 0.2, 0.2)

def get_adjacent_lenght(object_point, robot):
    # Points the gripper downwards to prepare the gripper to pick up object

    trans_z = object_point[2]
    angle =  robot.getl()[3]

    adjacent_length = math.cos(degrees(angle)) * trans_z

    print ('adjacent_length: ', adjacent_length)

    return adjacent_length

def find_frame_areas(boxes):
    return [box.bounding_boxes[0] for box in boxes]

def align_gripper(pipeline, model, robot):

    img = capture_image(pipeline)
    # rotate the gripper so it's aligned with the object
    image_rotation_angle = 1
    robot_rotation_angle = 0  # initialize robot_rotation_angle to 0
    smallest_frame_area = float('inf')  # set initial smallest_frame_area to be infinity
    while True:
        # Rotate the image
        rotation_matrix = cv2.getRotationMatrix2D((img.shape[1] // 2, img.shape[0] // 2), image_rotation_angle, 1.0)
        rotated_img = cv2.warpAffine(img, rotation_matrix, (img.shape[1], img.shape[0]))
        boxes = model(rotated_img, conf=0.01)[0].boxes
        frame_areas = find_frame_areas(boxes)
        
        current_frame_area = min(frame_areas)
        if current_frame_area < smallest_frame_area:
            smallest_frame_area = current_frame_area
            robot_rotation_angle = image_rotation_angle
        elif image_rotation_angle > 45 and smallest_frame_area < current_frame_area:
            break  # Break the loop when a small frame area is found
        image_rotation_angle += 1  # Increase image_rotation_angle for the next iteration
        
    robot.movej(robot.getj()[:-1] + [math.radians(robot_rotation_angle)], acc=0.2, vel=0.2)

def main():
    robot = connect_robot()
    model = load_model()
    pipeline = start_streaming()
    object_center = allign_object(pipeline, model)

    if object_center:
        object_point = center_the_gripper(robot, model, object_center, pipeline)
        print("OBJECT_POINT: " , object_point)

        move_over_object(object_point, robot)
        align_gripper(pipeline, model, robot)


if __name__ == "__main__":
    # main()
    robot = connect_robot()
    current_location = robot.getl()
    print(current_location)
    gripper_flat = current_location
    gripper_flat[3] = 3.14
    gripper_flat[4] = 0.0
    gripper_flat[5] = 0.0

    robot.movel(gripper_flat, acc = 0.5, vel = 0.2)
    # loc = robot.getl()
    # loc[0]-=0.5
    # robot.movel(loc, 0.2,0.2)

# # This updated code creates a new canvas (initialized with zeros) with the same dimensions as the original image. The rotated image is then pasted onto the canvas, considering its position within the canvas based on the center point.

# The resulting composite canvas will show the non-rotated image with the rotated image positioned correctly within it. You can modify the canvas if you prefer a different color or transparency for the padded area.

# Don't forget to replace "path_to_your_image.jpg" with the actual path to your image file."""