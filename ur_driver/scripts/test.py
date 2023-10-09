import cv2
import pyrealsense2 as rs
import time
import torch
import numpy as np
from torchvision.transforms import functional as F
from ultralytics import YOLO

model_file_path = '/home/rpl/wei_ws/src/ur_module/ur_driver/ur_driver/ur_tools/best.pt'
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
    depth = np.asanyarray(depth_frame.get_data())
    depth = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)

    # depth = cv2.resize(depth, (640, 480))


    # Convert the frame to the format expected by the model
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # Convert to BGR format
    img = cv2.resize(img, (640, 480))

    # Perform object detection on the frame
    boxes = model(img)[0].boxes  # Perform object detection
    # print(boxes.xyxy)
    # print(boxes.cls)

    for (xmin, ymin, xmax, ymax), cls in zip(boxes.xyxy, boxes.cls):
        depth_value = depth_frame.get_distance(int((xmin + xmax) / 2), int((ymin + ymax) / 2))
        distance = depth_value
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2) #calculate the center of the object
        offset_object_x = 320 - center_x
        offset_object_y = 240 - center_y

        # print("Offset from center (X, Y):", offset_object_x, offset_object_y)
        cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
        cv2.putText(img, f"{distance:.2f}m", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        # cv2.putText(img, f"{boxes.cls}", (int(xmin), int(ymin) -25), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line
        # cv2.circle(img, (320, 240), 5, (0, 0, 255), -1) #put the circle in the center with a filled in line

    # Display the color image with bounding boxes
    cv2.imshow("Object Detection", img)
    cv2.imshow("depth", depth)


    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the pipeline and release resources
pipeline.stop()
cv2.destroyAllWindows()