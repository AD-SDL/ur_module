import pyrealsense2 as rs
import numpy as np
import cv2
import time


pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # Color stream configuration
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream configuration
profile = pipeline.start(config) 
print(profile)

while True:
    # Wait for the next set of frames
    frames = pipeline.wait_for_frames()
    print(frames)

    # Get color and depth frames
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

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
