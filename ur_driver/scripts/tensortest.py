import cv2
import pyrealsense2 as rs
import time
import torch
import numpy as np
from torchvision.transforms import functional as F
from ultralytics import YOLO

model_file_path = '/home/rpl/Documents/best.pt'
# Load the trained YOLO model
model = YOLO(model_file_path)
# Set the desired objects to detect
desired_objects = ['wellplates', 'tipboxes', 'hammers', 'deepwellplates', 'wellplate_lids']  #list of known objects
# Initialize the Intel RealSense camera pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # Color stream configuration
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream configuration
profile = pipeline.start(config) 
# Start capturing and processing frames
rotation_angle = 0

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

        depth_intrin = depth_frame.profile.as_video_stream_profile().instrinsics
        
    
        offset_object_x = 320 - center_x
        offset_object_y = 240 - center_y

        print("Offset from center (X, Y):", offset_object_x, offset_object_y)

        cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
        cv2.putText(img, f"{distance:.2f}m", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line
        cv2.circle(img, (320, 240), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line
        
    # for (xmin, ymin, xmax, ymax), cls in zip(find_angle.xyxy, find_angle.cls):
           
    #         center_x = int((xmin + xmax) / 2)
    #         center_y = int((ymin + ymax) / 2) #calculate the center of the object
    #         offset_object_x = 320 - center_x
    #         offset_object_y = 240 - center_y

    #         print("Offset from center (X, Y):", offset_object_x, offset_object_y)

            # cv2.rectangle(rotated_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
            # cv2.circle(rotated_img, (center_x, center_y), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line
            # cv2.circle(rotated_img, (320, 240), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line

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
    cv2.imshow("Original Image", img)
    # cv2.imshow("Rotated Image", rotated_img)

    # cv2.imshow("Rotated Image", rotated_image)
    # cv2.imshow("Composite Canvas", canvas)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the pipeline and release resources
pipeline.stop()
cv2.destroyAllWindows()






