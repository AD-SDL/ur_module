"""Controls Various Type of Gripper End Effectors"""

from copy import deepcopy
from time import sleep
from typing import Union

from madsci.common.types.location_types import LocationArgument

from .robotiq_gripper_driver import RobotiqGripper


class FingerGripperController:
    """Controls Robotiq Finger Grippers"""

    def __init__(
        self,
        hostname: str = "146.137.240.38",
        port: int = 63352,
        ur=None,
        resource_client=None,
        gripper_resource_id: str = None,
    ):
        """
        Constructor for the FingerGripperController class.

        Args:
            hostname (str): The hostname of the robot.
            port (int): Port number to connect to the robot over the Interpreter socket
        """
        self.host = hostname
        self.PORT = port

        if not ur:
            raise Exception("UR connection is not established")
        else:
            self.ur = ur
            # self.ur.set_payload(1.2)# TODO: Check the actual payload

        self.gripper_close = 255  # 0-255 (255 is closed)
        self.gripper_open = 0
        self.gripper_speed = 255  # 0-255
        self.gripper_force = 255  # 0-255

        self.acceleration = 0.7
        self.velocity = 0.7
        self.speed_ms = 0.750
        self.speed_rads = 0.750
        self.accel_mss = 1.200
        self.accel_radss = 1.200
        self.blend_radius_m = 0.001
        self.ref_frame = [0, 0, 0, 0, 0, 0]

    def connect_gripper(self):
        """
        Connect to the gripper
        """
        for i in range(2):
            try:
                # GRIPPER SETUP:
                self.gripper = RobotiqGripper()
                print("Connecting to gripper...")
                self.gripper.connect(hostname=self.host, port=self.PORT)

                if self.gripper.is_active():
                    print("Gripper already active")
                else:
                    print("Activating gripper...")
                    self.gripper.activate()
                    print("Opening gripper...")
                    self.open_gripper()

            except Exception as err:
                print("Gripper connection failed, try {}: {} ".format(i + 1, err))
                self.ur.set_tool_communication(
                    baud_rate=115200,
                    parity=0,
                    stop_bits=1,
                    rx_idle_chars=1.5,
                    tx_idle_chars=3.5,
                )
                sleep(4)

            else:
                print("Gripper is ready!")

    def disconnect_gripper(self):
        """
        Discconect from the gripper
        """
        try:
            self.gripper.disconnect()
        except Exception as err:
            print("Gripper error: ", err)

        else:
            print("Gripper connection is closed")

    def home_robot(self, home: Union[LocationArgument, list] = None) -> None:
        """
        Home the robot
        """
        if not home:
            return
        if isinstance(home, LocationArgument):
            home_location = home.location
        elif isinstance(home, list):
            home_location = home
        self.ur.movej(home_location, self.acceleration, self.velocity)

    def open_gripper(
        self,
        pose: float = None,
        speed: float = None,
        force: float = None,
    ) -> None:
        """Opens the gripper using pose, speed and force variables"""
        if pose:
            self.gripper_open = pose
        if force:
            self.gripper_force = force
        if speed:
            self.gripper_speed = speed

        self.gripper.move_and_wait_for_pos(
            self.gripper_open,
            self.gripper_speed,
            self.gripper_force,
        )
        sleep(0.5)

    def close_gripper(
        self,
        pose: float = None,
        speed: float = None,
        force: float = None,
    ) -> None:
        """Closes the gripper using pose, speed and force variables"""
        if pose:
            self.gripper_close = pose
        if force:
            self.gripper_force = force
        if speed:
            self.gripper_speed = speed

        self.gripper.move_and_wait_for_pos(
            self.gripper_close,
            self.gripper_speed,
            self.gripper_force,
        )
        sleep(0.5)

    def pick(
        self,
        source: Union[LocationArgument, list] = None,
        approach_axis: str = None,
        approach_distance: float = None,
    ):
        """Pick up from first goal position"""

        if isinstance(source, LocationArgument):
            source_location = source.location
        elif isinstance(source, list):
            source_location = source
        else:
            raise Exception("Please provide an accurate source loaction")

        if not approach_distance:
            approach_distance = 0.05

        axis = None

        if not approach_axis or approach_axis.lower() == "z":
            axis = 2
        elif approach_axis.lower() == "y":
            axis = 1
        elif approach_axis.lower() == "-y":
            axis = 1
            approach_distance = -approach_distance
        elif approach_axis.lower() == "x":
            axis = 0
        elif approach_axis.lower() == "-x":
            axis = 0
            approach_distance = -approach_distance

        above_goal = deepcopy(source_location)
        above_goal[axis] += approach_distance

        self.open_gripper()

        print("Moving to above goal position")
        self.ur.movel(above_goal, self.acceleration, self.velocity)

        print("Moving to goal position")
        self.ur.movel(source_location, self.acceleration, self.velocity)

        print("Closing gripper")
        self.close_gripper()

        if self.resource_client and isinstance(source, LocationArgument):  # Handle resources if configured
            popped_object, updated_resource = self.resource_client.pop(resource=source.resource_id)
            self.resource_client.push(resource=self.gripper_resource_id, child=popped_object)

        print("Moving back to above goal position")
        self.ur.movel(above_goal, self.acceleration, self.velocity)

    def place(
        self,
        target: Union[LocationArgument, list] = None,
        approach_axis: str = None,
        approach_distance: float = None,
    ):
        """Place down at second goal position"""

        if isinstance(target, LocationArgument):
            target_location = target.location
        elif isinstance(target, list):
            target_location = target
        else:
            raise Exception("Please provide an accurate target loaction")

        if not approach_distance:
            approach_distance = 0.05

        axis = None

        if not approach_axis or approach_axis.lower() == "z":
            axis = 2
        elif approach_axis.lower() == "y":
            axis = 1
        elif approach_axis.lower() == "-y":
            axis = 1
            approach_distance = -approach_distance
        elif approach_axis.lower() == "x":
            axis = 0
        elif approach_axis.lower() == "-x":
            axis = 0
            approach_distance = -approach_distance

        above_goal = deepcopy(target_location)
        above_goal[axis] += approach_distance

        print("Moving to above goal position")
        self.ur.movel(above_goal, self.acceleration, self.velocity)

        print("Moving to goal position")
        self.ur.movel(target_location, self.acceleration, self.velocity)

        print("Opennig gripper")
        self.open_gripper()

        if self.resource_client and isinstance(target, LocationArgument):  # Handle resources if configured
            popped_object, updated_resource = self.resource_client.pop(resource=self.gripper_resource_id)
            self.resource_client.push(resource=target.resource_id, child=popped_object)
        print("Moving back to above goal position")
        self.ur.movel(above_goal, self.acceleration, self.velocity)

    def transfer(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        source_approach_axis: str = None,
        target_approach_axis: str = None,
        source_approach_distance: float = None,
        target_approach_distance: float = None,
    ) -> None:
        """Handles the transfer request"""
        self.pick(
            source=source,
            approach_axis=source_approach_axis,
            approach_distance=source_approach_distance,
        )
        print("Pick up completed")
        self.home_robot(home=home)
        self.place(
            place=target,
            approach_axis=target_approach_axis,
            approach_distance=target_approach_distance,
        )
        print("Place completed")


class VacuumGripperController:
    """Robotiq Vacuum Gripper Controller"""

    def __init__(
        self,
        IP: str = "146.137.240.38",
        PORT: int = 29999,
        gripper: bool = False,
    ):
        """Constructor for VacummGripperController"""
        super().__init__(IP=IP, PORT=PORT)
