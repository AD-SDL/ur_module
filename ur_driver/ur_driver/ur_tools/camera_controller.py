from time import sleep, time
from copy import deepcopy
from typing import Optional, Tuple, List

import cv2
import pyrealsense2 as realsense

from math import cos, degrees, radians

import torch
from torchvision.transforms import functional as F
from ultralytics import YOLO
import numpy as np

from robotiq_gripper_driver import RobotiqGripper
from urx import Robot


class CameraController:
    """
    CameraController is a class created for UR robots to utilize Intel Realsense D400 series cameras.
    It can detect objects based on a pre-trained YOLO model with 5 different labwares, and plan
    a robot trajectory to pick up the objects based on distance and object reference frame information obtained from the camera.
    """

    def __init__(self, robot_ip: str, ur_connection: Optional[Robot], target_object: Optional[str]) -> None:
        """
        Constructor for the CameraController class.

        Args:
            robot_IP (str): The IP address of the robot.
            ur_connection (Robot): The connection to the robot (urx.Robot instance), defaults to None.
            target_object (str): The target object for YOLO model, defaults to None.

        Raises:
            ValueError: Raised when ur_connection or target_object is not provided.
            ValueError: Raised when target_object category doesn't exist in the trained model class list.
        """
        if ur_connection is None:
            raise ValueError("UR connection is not provided!")
        if target_object is None:
            raise ValueError("Target object for YOLO model is not provided")

        self.ur_connection = ur_connection
        self.target_object = target_object.lower()
        self.model = None
        self.object_distance = 0
        self.object_reference_frame = None
        self.gripper = RobotiqGripper()

        self.MOVE_ACC = 0.5
        self.MOVE_VEL = 0.5
        self.conveyor_speed = 0.0  # Conveyor speed in meters per second
        self.CLASS_NAMES = ['deepwellplates', 'tipboxes', 'hammers', 'wellplates', 'wellplate_lids']

        self._validate_target_object()
        self._connect_to_gripper(robot_ip)

    def _validate_target_object(self):
        if self.target_object not in self.CLASS_NAMES:
            raise ValueError(f"Target object category '{self.target_object}' doesn't exist in the trained model class list")

    def _connect_to_gripper(self, robot_ip: str):
        print('Connecting to gripper...')
        self.gripper.connect(robot_ip, 63352)
        self.gripper.activate()
        self.gripper.move_and_wait_for_pos(0, 150, 0)

    def start_camera_stream(self) -> None:
        """Start the Intel realsense camera pipeline"""

        try:
            self.pipeline = realsense.pipeline()
            config = realsense.config()
            config.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30)
            config.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30)
            self.pipeline.start(config)
        except realsense.error as e:
            print(f"RealSense error {e.get_failed_function()}: {e.get_failed_args()}")
            print(f"{e.get_description()}")    

    def capture_image(self) -> Tuple[np.array, 'realsense.frame', 'realsense.frame']:
        """
        Capture a new image from the camera.

        Returns:
            Tuple[np.array, 'realsense.frame', 'realsense.frame']: The captured image and the color and depth frames.
        """
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        img = np.asanyarray(color_frame.get_data())
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (640, 480))

        return img, color_frame, depth_frame

    def load_yolo_model(self, model_path: Optional[str] = None):
        """
        Loads the trained YOLO model.

        Args:
            model_path (Optional[str]): Path to the model file, defaults to None.
        """
        model_file_path = model_path if model_path else "best.pt"
        # Load the trained YOLO model
        self.model = YOLO(model_file_path)

    def _get_object_predictions(self, img: np.array) -> Tuple[list, list]:
        """
        Detects objects in the given image using the provided YOLO model.

        Args:
            model (YOLO): The YOLO model to use for object detection.
            img (np.array): The image to detect objects in.

        Returns:
            Tuple[list, list]: A tuple containing a list of detected boxes and their corresponding classes.
        """
        prediction = self.model(img)[0]
        boxes = prediction.boxes
        classes = prediction.cls

        return boxes, classes
    
    # This function needs to be implemented:
    def object_is_within_threshold(self, object_point: Tuple[float, float, float]) -> bool:
        """
        Determines whether the object is within a certain distance threshold for picking up.
        
        Args:
            object_point (Tuple[float, float, float]): The 3D coordinates of the object point.

        Returns:
            bool: True if the object is within the threshold, otherwise False.
        """
        threshold = 0.1  # This is an example value and should be adjusted based on your specific requirements
        distance = np.linalg.norm(np.array(object_point))
        return distance <= threshold
    
    def _calculate_object_xy(self, boxes) -> Optional[Tuple[int, int]]:
        """
        Calculates the center of the first detected object if any objects are detected.

        Args:
            boxes (List[Box]): A list of detected objects represented as boxes.

        Returns:
            Optional[Tuple[int, int]]: The x and y coordinates of the first object's center if detected, otherwise None.
        """
        if len(boxes) > 0:
            xmin, ymin, xmax, ymax = boxes[0].xyxy[0]
            center_x = int((xmin + xmax) / 2)
            center_y = int((ymin + ymax) / 2)
            return center_x, center_y
        else:
            return None
        
    def get_object_xy(self, timeout = 10) -> Optional[Tuple[int, int]]:
        """
        Aligns the robot arm to the object's center until the center is detected.
        
        Args:
            timeout (int): The maximum amount of time (in seconds) to try aligning the robot arm.

        Returns:
            Optional[Tuple[int, int]]: The x and y coordinates of the object's center if detected, otherwise None.
        """
        object_xy = None
        start_time = time()
        while object_xy is None and time() - start_time < timeout:
            img, color_frame, depth_frame = self.capture_image()

            if not color_frame or not depth_frame or not img:
                return None

            boxes, classes = self._get_object_predictions(img)
            object_xy = self._calculate_object_xy(boxes)

        return object_xy
        
    def _calculate_object_reference_frame(self, depth_frame: 'realsense.frame', center_x: int, center_y: int):
        """
        Get the object reference frame from the depth frame and center of the bounding box.

        Args:
            depth_frame (frame): Depth frame of the image.
            center_x (int): X coordinate of the object center.
            center_y (int): Y coordinate of the object center.
        """

        # Get the intrinsic parameters of the depth frame
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        # Use the depth value and the center of the bounding box to get the 3D coordinates of the object
        self.object_reference_frame = realsense.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], self.object_distance)
    
    @staticmethod
    def _calculate_box_center(xmin: float, xmax: float, ymin: float, ymax: float) -> Tuple[int, int]:
        return int((xmin + xmax) / 2), int((ymin + ymax) / 2)

    @staticmethod
    def _draw_on_image(img: np.array, xmin: float, ymin: float, xmax: float, ymax: float, center_x: int, center_y: int, distance: float) -> None:
        cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
        cv2.putText(img, f"{distance:.2f}m", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.circle(img, (640/2, 480/2), 5, (0, 0, 255), -1)

    def move_to_object(self, move_z: Optional[float] = 0.0):
        """Method to move the robot arm to the object"""

        # Extract the x, y, and z coordinates
        trans_x = self.object_reference_frame[0]
        trans_y = self.object_reference_frame[1]
        trans_z = self.object_reference_frame[2]

        # Print out the object's 3D coordinates
        print("Object XYZ: " + str(self.object_reference_frame))

        if trans_z != 0:
            # Move the robot's tool (e.g. a gripper) to the object
            self.ur_connection.translate_tool([-trans_x, -trans_y, move_z], acc=self.MOVE_ACC, vel=self.MOVE_VEL)
  
    def _detect_and_allign(self, img: np.array, depth_frame: 'realsense.frame'):
        """
        This function takes an image and a depth frame as input, runs the object detection model on the image,
        and checks the classes of detected objects. If the class of a detected object matches the target object,
        it calculates the center of this object, gets its distance, and draws on the image. Then, it calculates 
        the object reference frame based on the center of the object and depth frame and commands the robot to 
        move to this object. If the target object is not found in the image, it logs a message.

        Args:
            img (np.array): The image from the camera.
            depth_frame ('realsense.frame'): The depth frame from the camera.
        """

        boxes, classes = self._get_object_predictions(img)
        for (xmin, ymin, xmax, ymax), cls in zip(boxes.xyxy, classes):
            if cls == self.target_object:
                center_x, center_y = self._calculate_box_center(xmin, xmax, ymin, ymax)
                self.object_distance = depth_frame.get_distance(center_x, center_y)

                self._draw_on_image(img, xmin, ymin, xmax, ymax, center_x, center_y, self.object_distance)

                self._calculate_object_reference_frame(depth_frame, center_x, center_y)
                break
        else:
            raise Exception(f'Target object {self.target_object} not found in the frame.')

    def center_the_gripper(self) -> None:
        """
        Method to center the robot gripper over the detected object in its field of view.
        
        Args:
            object_xy (tuple): Optional; pre-calculated object center coordinates.

        Returns:
            None
        """


        # Capture image and get color and depth frames
        img, color_frame, depth_frame = self.capture_image()
        if not color_frame or not depth_frame or not img:
            raise ValueError("Could not capture image or retrieve color/depth frames")

        self._detect_and_allign(img, depth_frame)

    def move_over_object(self, object_point: Tuple[float, float, float]) -> None:
        """
        Moves the robot arm over the detected object.

        Args:
            object_point (Tuple[float, float, float]): The 3D coordinates of the object point.
        """
        adjacent_length = self._get_adjacent_length(object_point)

        self._move_gripper_perpendicular()

        desired_position = adjacent_length 
        self.ur_connection.translate_tool([0, desired_position , 0], 1, 0.2)

    def _get_adjacent_length(self, object_point: Tuple[float, float, float]) -> float:
        """
        Calculates the adjacent length to the object from the robot arm.

        Args:
            object_point (Tuple[float, float, float]): The 3D coordinates of the object point.

        Returns:
            float: The calculated adjacent length.
        """
        trans_z = object_point[2]
        angle = self.ur_connection.getl()[4]

        adjacent_length = cos(degrees(angle)) * trans_z
        print(f'Adjacent length: {adjacent_length}')

        return abs(adjacent_length)

    def _move_gripper_perpendicular(self) -> None:
        """
        Adjusts the gripper to a perpendicular orientation.
        """
        current_orientation = self.ur_connection.get_orientation()
        euler_angles = current_orientation.to_euler(encoding="xyz")

        move_rx = (3.14 - abs(euler_angles[0]))
        move_ry = abs(euler_angles[1])

        current_orientation.rotate_xt(move_rx)
        current_orientation.rotate_yt(move_ry)

        self.ur_connection.set_orientation(current_orientation, 0.2, 0.2)

    def find_frame_areas(self, boxes) -> List[float]:
        """
        Determines the areas of bounding boxes of the detected objects.
        
        Args:
            boxes (List[Box]): A list of detected objects represented as boxes.
            
        Returns:
            List[float]: A list of areas of each bounding box.
        """
        areas = []
        for box in boxes:
            xmin, ymin, xmax, ymax = box.xyxy[0]
            areas.append((xmax - xmin) * (ymax - ymin))  # Area = width * height
        return areas
    
    def align_gripper(self) -> None:
        """
        Rotates the image and aligns the gripper with the target object until a minimum bounding box area is found.
        
        Returns:
            None
        """
        img, _, _ = self.capture_image()
        image_rotation_angle = 1
        robot_rotation_angle = 0  
        smallest_frame_area = float('inf')  

        rotate = True  # introduce a flag to control the while loop

        while rotate:
            rotation_matrix = cv2.getRotationMatrix2D((img.shape[1] // 2, img.shape[0] // 2), image_rotation_angle, 1.0)
            rotated_img = cv2.warpAffine(img, rotation_matrix, (img.shape[1], img.shape[0]))
            
            boxes, classes = self._get_object_predictions(rotated_img)
            for i, cls in enumerate(classes):
                if cls != self.target_object:
                    continue
                frame_areas = self.find_frame_areas([boxes[i]])

                current_frame_area = min(frame_areas)
                if current_frame_area < smallest_frame_area:
                    smallest_frame_area = current_frame_area
                    robot_rotation_angle = image_rotation_angle
                elif image_rotation_angle > 45 and smallest_frame_area < current_frame_area:
                    rotate = False  # if the smallest frame area is found, stop the while loop
                    break
            image_rotation_angle += 1

        self.ur_connection.movej(self.ur_connection.getj()[:-1] + [radians(robot_rotation_angle)], acc=0.2, vel=0.2)

    def pick_static_object(self):
        """
        Detects, aligns to, and picks up an object using the robot arm and gripper.
        """

        for i in range(6):
            object_xy = self.get_object_xy()

            if object_xy:
                self.center_the_gripper()
                print(f"OBJECT_POINT: {self.object_reference_frame}")
                self.move_to_object()
            else:
                self.object_reference_frame = None

        if not self.object_reference_frame:
            print("Object can't be found!")
            return
        
        sleep(4)

        self.ur_connection.translate_tool([0.02, 0.09, 0], 1, 0.2)
        self.ur_connection.translate_tool([0, 0, self.object_reference_frame[2]-0.16], 1, 0.2)
        self.gripper.move_and_wait_for_pos(160, 150, 100)
        self.ur_connection.translate_tool([0, 0, -(self.object_reference_frame[2]-0.2)], 1, 0.2)

        waypoint = [0.2655990719795227, -1.7126232586302699, -1.7399795055389404, -1.254279003744461, -4.749170009289877, -2.394965473805563]
        drop_off_above = [-1.4304960409747522, -1.0302266043475647, -2.2368316650390625, -1.4599171516350289, -4.7227471510516565, -3.00033146539797]
        drop_off = [-1.4450705687152308, -1.3130722504905243, -2.613124132156372, -0.8007843655398865, -4.7251179854022425, -3.009803597127096]

        self.ur_connection.movej(waypoint, 0.5, 0.5)
        self.ur_connection.movej(drop_off_above, 0.5, 0.5)
        self.ur_connection.movej(drop_off, 0.5, 0.5)

        self.gripper.move_and_wait_for_pos(0, 150, 100)

        self.ur_connection.movej(drop_off_above, 0.5, 0.5)
        self.ur_connection.movej(waypoint, 0.5, 0.5)

    def pick_dynamic_object(self):
        """
        Detects, aligns to, and picks up an object using the robot arm and gripper.
        """
        object_grasped = False
        while not object_grasped:
            object_xy = self.get_object_xy()

            if object_xy:
                self.center_the_gripper()
                print(f"OBJECT_POINT: {self.object_reference_frame}")
                i
                self.move_to_object()

            else:
                self.object_reference_frame  = None

        if not self.object_reference_frame:
            print("Object can't be found!")
            return
        
        sleep(4)

        self.ur_connection.translate_tool([0.02, 0.09, 0], 1, 0.2)
        self.ur_connection.translate_tool([0, 0, self.object_reference_frame[2]-0.16], 1, 0.2)
        self.gripper.move_and_wait_for_pos(160, 150, 100)
        self.ur_connection.translate_tool([0, 0, -(self.object_reference_frame[2]-0.2)], 1, 0.2)

    def pick_conveyor_object(self):
        # TODO: Maybe keep alligning and try to pick up at the same time till object is actually picked up.
        #       When the align object is called, robot arm can also be move towards thte object slowly (0.1 each time) within each loop
        #       Once robot gets to certain distance it would only perform pick movement quickly.
        

        MAX_ATTEMPTS = 5

        while True:
            object_xy = self.get_object_xy()

            if object_xy:
                self.center_the_gripper(object_xy)

                if self.conveyor_speed != 0:
                    # If the conveyor belt is moving, adjust the target point based on its speed
                    pickup_delay = self._estimate_pickup_delay(self.object_reference_frame)  # Calculate the delay before the robot arm can pick up the object
                    self.object_reference_frame[1] += self.conveyor_speed * pickup_delay  # Adjust the target y-coordinate based on the conveyor speed and pickup delay

                self.move_over_object(self.object_reference_frame)

            sleep(5)

            self.align_gripper()
            self.grip_object()

            if self.has_picked_up_object():
                break

            num_failed_attempts += 1
            if num_failed_attempts > MAX_ATTEMPTS:
                print("Failed to pick up object after multiple attempts.")
                break

    def _estimate_pickup_delay(self, object_point: Tuple[float, float, float]) -> float:
        """
        Estimates the time it will take for the robot arm to reach the target point.
        For simplicity, this example assumes a constant time delay. In a real application, you might want to calculate this based on the robot's current state and the target point.
        """
        robot_position = self.ur_connection.getl()
        distance_to_target = ((robot_position[0]-object_point[0])**2 + (robot_position[1]-object_point[1])**2 + (robot_position[2]-object_point[2])**2)**0.5
        time = distance_to_target / self.MOVE_VEL
        return time    
    
    # Uncompleted
    def calculate_3d_orientation(self, depth_frame):
        # Obtain depth data
        depth_data = np.asanyarray(depth_frame.get_data())
        # Assuming that we already have the bounding box of the object
        box = 1 # Bounding box of the detected object
        # Extract the depth information for the object from the depth data
        object_depth_data = depth_data[box[1]:box[3], box[0]:box[2]]
        # Find the 3D orientation based on the object's depth data
        orientation = 1 # Function to calculate orientation based on depth data
        # return orientation
        pass

    # Uncompleted
    def align_gripper_to_object(self):
        # Capture image
        _, _, depth_frame = self.capture_image()
        # Calculate 3D orientation of the object
        orientation = self.calculate_3d_orientation(depth_frame)
        # Align the gripper to the object's 3D orientation
        self.ur_connection.set_orientation(orientation, acc=0.2, vel=0.2)
        pass
    
def main():
    # Initialize CameraController
    robot_ip = '192.168.1.10'  # replace with your robot's IP
    ur_robot = Robot(robot_ip)  # Initialize the UR robot connection
    target_object = 'wellplates'  # replace with the object you want to pick
    controller = CameraController(robot_ip, ur_robot, target_object)
    
    # Load model and start streaming
    controller.load_yolo_model('best.pt')  # replace with your model's path
    controller.start_camera_stream()

    for i in range(6):
        object_xy = controller.get_object_xy()

        if object_xy:
            controller.center_the_gripper(object_xy)
            print("OBJECT_POINT: ", controller.object_reference_frame)

    controller.move_over_object(controller.object_reference_frame)
    sleep(5)

    controller.align_gripper()
    controller.pick_static_object()
    controller.pipeline.stop()

if __name__ == "__main__":
    main()