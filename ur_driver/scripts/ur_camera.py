from math import cos, degrees
import time

import numpy as np

import cv2
import pyrealsense2 as rs

from torchvision.transforms import functional as F
import torch
from ultralytics import YOLO


class URCamera():
    def __init__(self ,ur_connection = None) -> None:

        self.robot = ur_connection
    
    def start_camera(self):
        pass

    def start_object_scan(self):
        pass

    def detect_objects(self):
        pass

    def calculate_object_distace(self):
        pass

    def rotate_image(self):
        pass

    def align_gripper(self):
        pass
    
    def move_object_top(self):
        pass

if __name__ == "__main__":

    ur_camera = URCamera()
    ur_camera.start_camera()
    ur_camera.start_object_scan()

    