"""Controls Various Type of Gripper End Effectors"""

from copy import deepcopy
from math import radians
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
        self.resource_client = resource_client
        self.gripper_resource_id = gripper_resource_id

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

    def pick_screw(
        self,
        screw_loc: Union[LocationArgument, list] = None,
    ) -> None:
        """Handles the pick screw request"""

        if isinstance(screw_loc, LocationArgument):
            source_location = screw_loc.location
        elif isinstance(screw_loc, list):
            source_location = screw_loc

        above_goal = deepcopy(source_location)
        above_goal[2] += 0.06
        self.ur.movel(above_goal, self.acceleration, self.velocity)
        self.ur.movel(source_location, 0.2, 0.2)
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

    def place_screw(
        self,
        target: Union[LocationArgument, list] = None,
        screw_time: float = 9,
    ) -> None:
        """Handles the place screw request"""
        # Move to the target location

        if isinstance(target, LocationArgument):
            target_location = target.location
        elif isinstance(target, list):
            target_location = target

        above_target = deepcopy(target_location)
        above_target[2] += 0.03
        self.ur.movel(above_target, self.acceleration, self.velocity)
        self.ur.movel(target_location, 0.2, 0.2)

        target_pose = [0, 0, 0.00021, 0, 0, 3.14]  # Setting the screw drive motion
        print("Screwing down")

        self.ur.speedl_tool(
            target_pose, 2, screw_time
        )  # This will perform screw driving motion for defined number of seconds
        sleep(screw_time)
        print("Screw drive motion completed")

        self.ur.translate_tool([0, 0, -0.03], 0.5, 0.5)

    def remove_cap(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
    ) -> None:
        """Handles the remove cap request"""
        self.open_gripper()
        if isinstance(source, LocationArgument):
            source_location = source.location
        elif isinstance(source, list):
            source_location = source

        above_goal = deepcopy(source_location)
        above_goal[2] += 0.06
        self.ur.movel(above_goal, self.acceleration, self.velocity)
        self.ur.movel(source_location, 0.2, 0.2)

        self.close_gripper()

        target_pose = [0, 0, -0.001, 0, 0, -3.14]  # Setting the screw drive motion
        print("Removing cap")
        screw_time = 7
        self.ur.speedl_tool(
            target_pose, 2, screw_time
        )  # This will perform screw driving motion for defined number of seconds
        sleep(screw_time + 0.5)
        self.ur.translate_tool([0, 0, -0.03], 0.5, 0.5)

        self.home_robot(home)
        self.place(place_goal=target)
        self.home_robot(home)

    def place_cap(
        self,
        home: Union[LocationArgument, list] = None,
        source: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
    ) -> None:
        """Handles the replace cap request"""

        self.pick(pick_goal=source)
        self.home_robot(home)

        if isinstance(target, LocationArgument):
            target_location = target.location
        elif isinstance(target, list):
            target_location = target

        above_goal = deepcopy(target_location)
        above_goal[2] += 0.06
        self.ur.movel(above_goal, self.acceleration, self.velocity)
        self.ur.movel(target_location, 0.1, 0.1)

        # self.close_gripper()

        target_pose = [0, 0, 0.0001, 0, 0, 2.10]  # Setting the screw drive motion
        print("Placing cap")
        screw_time = 6
        self.ur.speedl_tool(
            target_pose, 2, screw_time
        )  # This will perform screw driving motion for defined number of seconds
        sleep(screw_time)

        self.open_gripper()
        self.ur.translate_tool([0, 0, -0.03], 0.5, 0.5)
        self.home_robot(home)

    def flip_object(
        self,
        target: Union[LocationArgument, list] = None,
        approach_axis: str = None,
    ) -> None:
        """Flips the object at the target location"""

        self.pick(pick_goal=target, approach_axis=approach_axis)

        cur_j = self.ur.getj()
        rotate_j = cur_j
        rotate_j[5] += radians(180)
        self.ur.movej(rotate_j, 0.6, 0.6)

        cur_l = self.ur.getl()
        target[3] = cur_l[3]
        target[4] = cur_l[4]
        target[5] = cur_l[5]

        self.place(place_goal=target, approach_axis=approach_axis)

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
            target=target,
            approach_axis=target_approach_axis,
            approach_distance=target_approach_distance,
        )
        print("Place completed")

    def screw_transfer(
        self,
        home: Union[LocationArgument, list] = None,
        target: Union[LocationArgument, list] = None,
        screwdriver_loc: Union[LocationArgument, list] = None,
        screw_loc: Union[LocationArgument, list] = None,
        screw_time: float = 9,
    ) -> None:
        """Handles the transfer request"""

        self.pick(
            pick_goal=screwdriver_loc,
        )  # Pick up the screwdriver bit
        self.home_robot(home=home)  # Move back to home position
        self.pick_screw(home=home, screw_loc=screw_loc)  # Pick up the screw
        self.place_screw(home=home, target=target, screw_time=screw_time)  # Drive the screwdriving motion
        self.home_robot(home=home)  # Move back to home position
        self.place(place_goal=screwdriver_loc)  # Place the screwdriver bit
        self.home_robot(home=home)


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
