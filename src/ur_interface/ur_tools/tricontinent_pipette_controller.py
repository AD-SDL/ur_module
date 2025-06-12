"""Tricontinent Pipette Controller Class"""

from copy import deepcopy
from time import sleep

from .pipette_driver import PipetteDriver


class TricontinentPipetteController:
    """This class aims to control tricontinent pipetes by utilizing a socket connection with the pipette over PipetteDriver class while controlling the UR robot simultaneously"""

    def __init__(
        self,
        hostname=None,
        ur=None,
        pipette_ip: str = None,
        resource_client=None,
        pipette_resource_id: str = None,
    ) -> None:
        """
        Initializes the PipetteController class.

        Parameters:
        - pipette_pv (str): The EPICS process variable (PV) for the pipette.
        - ur_connection: The connection object for the Universal Robot (UR) robot.
        """

        self.hostname = hostname
        self.resource_client = resource_client
        self.pipette_resource_id = pipette_resource_id

        if not ur or not pipette_ip:
            raise Exception("UR connection is not established")
        else:
            self.ur = ur
            self.IP = pipette_ip

        self.acceleration = 0.5
        self.speed_fast = 0.750
        self.speed_slow = 0.1
        self.ref_frame = [0, 0, 0, 0, 0, 0]

        self.pipette_drop_tip_value = -8
        self.pipette_aspirate_value = 2.0
        self.pipette_dispense_value = -2.0
        self.droplet_value = 0.3

    def connect_pipette(self):
        """
        Connects to the pipette by first setting the correct tool communication parameters
        """
        for i in range(5):
            try:
                # Establishing a connection with the pipette on RS485 comminication
                self.pipette = PipetteDriver()
                comm_setting = self.pipette._comm_setting
                self.ur.set_tool_communication(
                    baud_rate=comm_setting["baud_rate"],
                    parity=comm_setting["parity"],
                    stop_bits=comm_setting["stop_bits"],
                    rx_idle_chars=comm_setting["rx_idle_chars"],
                    tx_idle_chars=comm_setting["tx_idle_chars"],
                )
                sleep(2)
                self.pipette.connect(hostname=self.IP)

            except Exception as err:
                print("Pipette connection error: ", err)

            else:
                print("Pipette is connected after {} tries".format(i))
                break

    def initialize_pipette(self):
        """
        Initializes the pipette by setting the correct tool communication parameters
        and connecting to the pipette.
        """
        try:
            self.pipette.initialize()
            sleep(2)

        except Exception as err:
            print("Pipette initialization error: ", err)

        else:
            print("Pipette is initialized")

    def disconnect_pipette(self):
        """
        Disconnect pipette
        """

        try:
            self.pipette.disconnect()
            # self.ur.set_tool_communication(enabled=False)

        except Exception as err:
            print("Pipette disconnection error: ", err)

        else:
            print("Pipette is disconnected")

    def pick_tip(self, tip_loc, x=0, y=0):
        """
        Description: Picks up a new tip from the first location on the pipette bin.
        """

        tip_approach = deepcopy(tip_loc)
        tip_approach[2] += 0.02
        tip_above = deepcopy(tip_loc)
        tip_above[2] += 0.1

        print("Picking up the first pipette tip...")

        self.ur.movel(tip_above, self.acceleration, self.speed_fast)
        # sleep(2)
        self.ur.movel(tip_approach, self.acceleration, self.speed_fast)
        # sleep(2)
        self.ur.movel(tip_loc, self.acceleration, self.speed_slow)
        # sleep(3)
        self.ur.movel(tip_approach, self.acceleration, self.speed_slow)
        # sleep(2)
        self.ur.movel(tip_above, self.acceleration, self.speed_fast)
        # sleep(2)
        print("Pipette tip successfully picked up")

    def transfer_sample(
        self,
        home: list = None,
        sample_aspirate: list = None,
        sample_dispense: list = None,
        vol: int = 10,
    ):
        """
        Description:
            - Makes a new sample on the 96 well plate.
            - Mixes to liquits in a single well and uses a new pipette tip for each liquid.
            - In order to mix the liquids together, pipette performs aspirate and dispense operation multiple times in the well that contains both the liquids.
        """
        print("Making a sample using two liquids...")

        # MOVE TO THE FIRT SAMPLE LOCATION

        sample_aspirate_above = deepcopy(sample_aspirate)
        sample_aspirate_above[2] += 0.05

        self.ur.movel(
            sample_aspirate_above,
            self.acceleration,
            self.speed_fast,
        )
        self.ur.movel(sample_aspirate, self.acceleration, self.speed_slow)

        # ASPIRATE FIRST SAMPLE
        self.pipette.aspirate(vol=vol)
        sleep(5)

        if self.resource_client:
            self.resource_client.increase_quantity(
                resource=self.pipette_resource_id,
                amount=vol,
            )

        self.ur.movel(sample_aspirate_above, self.acceleration, self.speed_slow)
        self.ur.movej(home, 1, 1)

        sample_dispense_above = deepcopy(sample_dispense)
        sample_dispense_above[2] += 0.02
        self.ur.movel(
            sample_dispense_above,
            self.acceleration,
            self.speed_fast,
        )
        self.ur.movel(sample_dispense, self.acceleration, self.speed_slow)

        self.pipette.dispense(vol=vol)
        sleep(5)
        if self.resource_client:
            self.resource_client.decrease_quantity(
                resource=self.pipette_resource_id,
                amount=vol,
            )
        self.ur.movel(
            sample_dispense_above,
            self.acceleration,
            self.speed_slow,
        )
        self.ur.movej(home, 1, 1)

    def pick_and_move(
        self,
        safe_waypoint: list = None,
        sample_loc: list = None,
        target: list = None,
        vol: int = 10,
    ):
        """
        Description:
            - Picks up a sample from the sample location and moves it to the target location.
            - The sample is picked up using the pipette and then moved to the target location but not dispensed.
        """

        sample_loc_above = deepcopy(sample_loc)
        sample_loc_above[2] += 0.05

        self.ur.movel(
            sample_loc_above,
            self.acceleration,
            self.speed_fast,
        )
        self.ur.movel(sample_loc, self.acceleration, self.speed_slow)

        # ASPIRATE FIRST SAMPLE
        self.pipette.aspirate(vol=vol)
        sleep(5)

        if self.resource_client:
            self.resource_client.increase_quantity(
                resource=self.pipette_resource_id,
                amount=vol,
            )

        self.ur.movel(sample_loc_above, self.acceleration, self.speed_fast)
        if safe_waypoint:
            self.ur.movel(safe_waypoint, self.acceleration, self.speed_fast)
        target_above = deepcopy(target)
        target_above2 = deepcopy(target)
        target_above[2] += 0.05
        # target_above2[2] += 0.01
        target_above2[2] += 0.030
        self.ur.movel(
            target_above,
            self.acceleration,
            self.speed_fast,
            wait=True,
        )
        self.ur.movel(
            target_above2,
            self.acceleration,
            self.speed_slow,
        )

    def dispense_and_retrieve(
        self,
        safe_waypoint: list = None,
        target: list = None,
        vol: int = 10,
    ):
        """
        Description:
            - Dispenses the sample at the target location and retrieves the pipette.
            - The sample is dispensed using the pipette and then the robot is moved back to the home position.
        """
        self.ur.movel(target, self.acceleration, self.speed_slow)

        # DISPENSE FIRST SAMPLE
        self.pipette.dispense(vol=vol)
        sleep(5)
        if self.resource_client:
            self.resource_client.decrease_quantity(
                resource=self.pipette_resource_id,
                amount=vol,
            )

        target_above = deepcopy(target)
        target_above[2] += 0.05
        self.ur.movel(
            target_above,
            self.acceleration,
            self.speed_fast,
            wait=True,
        )
        if safe_waypoint:
            self.ur.movel(safe_waypoint, self.acceleration, self.speed_fast)

    def create_droplet(self, droplet_loc):
        """
        Description:
            - Drives pipette to create a droplet.
            - Number of motor steps to create a droplet is stored in "self.droplet_value".
            - Pipette is controlled by RS485 commication.
        """
        print("Creating a droplet...")
        droplet_front = deepcopy(droplet_loc)
        droplet_front[1] += 0.1
        self.ur.movel(droplet_front, self.acceleration, self.speed_fast)
        self.ur.movel(droplet_loc, self.acceleration, self.speed_fast)
        sleep(1)
        self.pipette.dispense(vol=3)
        sleep(5)
        self.pipette.aspirate(vol=3)
        sleep(1)
        self.ur.movel(droplet_front, self.acceleration, self.speed_slow)

    def empty_tip(self, sample_loc):
        """
        Description:
            - Dispense all the sample inside the pipette.

        """
        print("Emtying the tip...")

        # MOVE TO THE FIRT SAMPLE LOCATION
        sample_above = deepcopy(sample_loc)
        sample_above[2] += 0.1

        self.ur.movel(sample_above, self.acceleration, self.speed_fast)
        self.ur.movel(sample_loc, self.acceleration, self.speed_fast)
        self.pipette.dispense(vol=25)
        sleep(2)
        self.ur.movel(sample_above, self.acceleration, self.speed_fast)

    def eject_tip(
        self,
        eject_tip_loc: list = None,
        approach_axis: str = "x",
        approach_distance: float = -0.005,
    ):
        """
        Description: Drops the pipette tip into trash bin
        """
        if approach_axis.lower() == "y":
            axis = 0
        else:
            axis = 1

        trash_above = deepcopy(eject_tip_loc)
        trash_front = deepcopy(eject_tip_loc)
        trash_above[2] += 0.1
        trash_front[axis] += approach_distance
        trash_front_above = deepcopy(trash_front)
        trash_front_above[2] += 0.1

        print("Droping tip to the trash bin...")
        # Move to the trash bin location
        self.ur.movel(trash_front_above, self.acceleration, self.speed_fast)
        self.ur.movel(trash_front, self.acceleration, self.speed_fast)
        self.ur.movel(eject_tip_loc, self.acceleration, self.speed_fast)
        self.ur.movel(trash_above, self.acceleration, self.speed_fast)


if __name__ == "__main__":
    from urx import Robot

    # r = Robot("164.54.116.129")
    ip = "192.168.1.102"
    r = Robot(ip)
    a = TricontinentPipetteController(ur=r, pipette_ip=ip)
    a.connect_pipette()
    sleep(5)
    a.pipette.initialize()
    a.pipette.aspirate(vol=5)
    sleep(5)
    a.disconnect_pipette()
    r.close()
