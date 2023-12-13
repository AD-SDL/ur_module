from time import sleep
from copy import deepcopy

from .pipette_driver import PipetteDriver

class TricontinentPipetteController():

    def __init__(self, hostname = None, ur = None):
        """
        Initializes the PipetteController class.
        
        Parameters:
        - pipette_pv (str): The EPICS process variable (PV) for the pipette.
        - ur_connection: The connection object for the Universal Robot (UR) robot.
        """

        self.hostname = hostname

        if not ur:
            raise Exception("UR connection is not established")
        else:
            self.ur = ur
            self.robot = self.ur.ur_connection

        self.acceleration = 0.5
        self.velocity = 0.5
        self.speed_ms    = 0.750
        self.speed_rads  = 0.750
        self.accel_mss   = 1.200
        self.accel_radss = 1.200
        self.blend_radius_m = 0.001
        self.ref_frame = [0,0,0,0,0,0]

        self.pipette_drop_tip_value = -8
        self.pipette_aspirate_value = 2.0
        self.pipette_dispense_value = -2.0
        self.droplet_value = 0.3

    def connect_pipette(self):
        """
        Connect pipette
        """

        try:
            # Establishing a connection with the pipette on RS485 comminication
            self.pipette = PipetteDriver()
            comm_setting = self.pipette._comm_setting
            self.robot.set_tool_communication(baud_rate=comm_setting["baud_rate"],
                                          parity=comm_setting["parity"],
                                          stop_bits=comm_setting["stop_bits"],
                                          rx_idle_chars=comm_setting["rx_idle_chars"],
                                          tx_idle_chars=comm_setting["tx_idle_chars"])        
            self.pipette.connect(hostname=self.IP)
            self.pipette.initialize()

        except Exception as err:
            print("Pipette error: ", err)

        else:
            print("Pipette is connected")

    def disconnect_pipette(self):
        """
        Disconnect pipette
        """

        try:
            # Closing the connection with the pipette on EPICS 
            self.pipette.disconnect()
            
        except Exception as err:
            print("Pipette error: ", err)

        else:
            print("Pipette is disconnected")

    def pick_tip(self, tip_loc, x=0, y=0):
        """
        Description: Picks up a new tip from the first location on the pipette bin.
        """

        tip_approach = deepcopy(tip_loc)
        tip_approach[2] += 0.02
        tip_above = deepcopy(tip_loc)
        tip_above[2] += 0.15

        print("Picking up the first pipette tip...")
        speed_ms = 0.100

        self.robot.movel(tip_above,self.accel_radss,self.speed_rads,0,0)
        sleep(2)
        speed_ms = 0.01
        self.robot.movel(tip_approach,self.accel_radss,self.speed_rads,0,0)
        sleep(2)    
        self.robot.movel(tip_loc,self.accel_mss,speed_ms,0,0)
        sleep(3)
        self.robot.movel(tip_approach,self.accel_mss,speed_ms,0,0)
        sleep(2)
        speed_ms = 0.1
        self.robot.movel(tip_above,self.accel_mss,speed_ms,0,0)
        sleep(2)
        print("Pipette tip successfully picked up")

    def transfer_sample(self, sample_loc, well_loc = None):
        
        """
        Description: 
            - Makes a new sample on the 96 well plate.
            - Mixes to liquits in a single well and uses a new pipette tip for each liquid.
            - In order to mix the liquids together, pipette performs aspirate and dispense operation multiple times in the well that contains both the liquids.
        """
        print("Making a sample using two liquids...")
        
        # MOVE TO THE FIRT SAMPLE LOCATION
        speed_ms = 0.1

        sample_above = deepcopy(sample_loc)
        sample_above[2] += 0.1

        self.robot.movel(sample_above,self.accel_mss,self.speed_ms)
        self.robot.movel(sample_loc,self.accel_mss,speed_ms)

        # ASPIRATE FIRST SAMPLE
        self.pipette.aspirate(vol=25)
        sleep(2)
        self.robot.movel(sample_above,self.accel_mss,speed_ms)

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
        self.robot.movel(droplet_front,self.accel_mss,self.speed_ms)
        self.robot.movel(droplet_loc,self.accel_mss,self.speed_ms)
        sleep(1)
        self.pipette.dispense(vol=3)
        sleep(5)
        self.pipette.aspirate(vol=3)
        sleep(1)
        self.robot.movel(droplet_front,self.accel_mss,self.speed_ms)

    def empty_tip(self, sample_loc):
        """
        Description: 
            - Dispense all the sample inside the pipette.
         
        """
        print("Emtying the tip...")
        
        # MOVE TO THE FIRT SAMPLE LOCATION
        speed_ms = 0.1
        sample_above = deepcopy(sample_loc)
        sample_above[2] += 0.1

        self.robot.movel(sample_above,self.accel_mss,self.speed_ms)
        self.robot.movel(sample_loc,self.accel_mss,speed_ms)
        self.pipette.dispense(vol=25)
        sleep(2)
        self.robot.movel(sample_above,self.accel_mss,speed_ms)

    def eject_tip(self, eject_tip_loc):        
        """
        Description: Drops the pipette tip into trash bin
        """
        trash_above = deepcopy(eject_tip_loc)
        trash_front = deepcopy(eject_tip_loc)
        trash_above[2] += 0.1
        trash_front[0] += 0.01
        trash_front_above = deepcopy(trash_front)
        trash_front_above[2] += 0.1

        print("Droping tip to the trash bin...")
        # Move to the trash bin location
        self.robot.movel(trash_front_above, self.accel_mss, self.speed_ms)
        self.robot.movel(trash_front, self.accel_mss, self.speed_ms)
        self.robot.movel(eject_tip_loc, self.accel_mss, self.speed_ms)
        self.robot.movel(trash_above, self.accel_mss, self.speed_ms)


if __name__ == "__main__":
    a = TricontinentPipetteController(hostname="164.54.116.129")


