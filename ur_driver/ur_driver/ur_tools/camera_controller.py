from time import sleep
from copy import deepcopy

import cv2
import pyrealsense2 as rs
import time
import torch
import numpy as np
from torchvision.transforms import functional as F
from ultralytics import YOLO
import math
from math import degrees
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper as gripper1
import sys
from robotiq_gripper_driver import RobotiqGripper
# setting path
from urx import Robot
from transforms3d import euler, quaternions

class CameraController():

    def __init__(self) -> None:
        pass