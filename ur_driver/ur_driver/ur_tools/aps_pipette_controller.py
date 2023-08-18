from time import sleep
from copy import deepcopy

from .pipette_driver import PipetteDriver

class ApsPipetteController():

    def __init__(self, ur_connection = None, IP = None):
        """
        Initializes the PipetteController class.
        
        Parameters:
        - pipette_pv (str): The EPICS process variable (PV) for the pipette.
        - ur_connection: The connection object for the Universal Robot (UR) robot.
        """

        self.IP = IP

        if not ur_connection:
            raise Exception("UR connection is not established")
        else:
            self.ur = ur_connection

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
            self.ur.set_tool_communication(baud_rate=comm_setting["baud_rate"],
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

        self.ur.movel(tip_above,self.accel_radss,self.speed_rads,0,0)
        sleep(2)
        speed_ms = 0.01
        self.ur.movel(tip_approach,self.accel_radss,self.speed_rads,0,0)
        sleep(2)    
        self.ur.movel(tip_loc,self.accel_mss,speed_ms,0,0)
        sleep(3)
        self.ur.movel(tip_approach,self.accel_mss,speed_ms,0,0)
        sleep(2)
        speed_ms = 0.1
        self.ur.movel(tip_above,self.accel_mss,speed_ms,0,0)
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
        # well_above = deepcopy(well_loc)
        # well_above[2] += 0.05

        self.ur.movel(sample_above,self.accel_mss,self.speed_ms)
        sleep(2)
        self.ur.movel(sample_loc,self.accel_mss,speed_ms)
        sleep(2)

        # ASPIRATE FIRST SAMPLE
        # self.aspirate_pipette() #TODO: ASPIRATE HERE
        self.pipette.dispense(vol=20)
        self.ur.movel(sample_above,self.accel_mss,speed_ms)
        sleep(1)

        # MOVE TO THE 1ST WELL
        # self.ur.movel(well_above,self.accel_mss,speed_ms)
        # sleep(1)
        # self.ur.movel(well_loc,self.accel_mss,speed_ms)
        # sleep(1)

        # # DISPENSE FIRST SAMPLE INTO FIRST WELL
        # self.dispense_pipette()
        # self.ur.movel(well_above,self.accel_mss,speed_ms)
        # sleep(1)

    def create_droplet(self):
        """
        Description: 
            - Drives pipette to create a droplet.
            - Number of motor steps to create a droplet is stored in "self.droplet_value".
            - Pipette is controlled by RS485 commication.
        """
        print("Creating a droplet...")
        self.pipette.dispense(vol=2)
        sleep(5)
        # self.pipette.aspirate(vol=2)

    def mix_samples(self, well_loc):

        well_above = well_loc
        well_above[2] += 0.05

         # MOVE TO THE 1ST WELL
        self.ur.movel(well_above,self.accel_mss,self.speed_ms,0,0)
        sleep(1)
        self.ur.movel(well_loc,self.accel_mss,self.speed_ms,0,0)
        sleep(1)

        # MIX SAMPLE
        for i in range(3):
            self.aspirate_pipette()
            self.dispense_pipette()

        # Aspirate all the liquid   
        self.aspirate_pipette()
        self.aspirate_pipette()

        self.ur.movel(well_above,self.accel_mss,self.speed_ms,0,0)
        sleep(1)
        print("Sample is prepared")


    def aspirate_pipette(self):
        """
        Description: 
            - Drives pipette to aspirate liquid. 
            - Number of motor steps to aspirate liquid is stored in "self.pipette_aspirate_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Aspirating the sample...")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) + self.pipette_aspirate_value)
        sleep(1)
    
    def drop_tip_to_trash(self):        
        """
        Description: Drops the pipette tip by driving the pipette all the way to the lowest point.
        """

        print("Droping tip to the trash bin...")
        # Move to the trash bin location
        self.ur.movel(self.trash_bin_above, self.accel_mss, self.speed_ms,0,0)
        sleep(2)
        self.ur.movel(self.trash_bin, self.accel_mss, self.speed_ms, 0, 0)
        sleep(2)
        self.eject_tip()
        sleep(1)
        self.ur.movel(self.trash_bin_above, self.accel_mss, self.speed_ms,0,0)
        sleep(2)

    def eject_tip(self):
        """
        Description: Ejects the pipette tip
        """
        print("Ejecting the tip")
        self.pipette.put(self.pipette_drop_tip_value)
        sleep(2)
        self.pipette.put(0)
        sleep(2)
  
    def empty_tip(self):
        """
        Description: Dispenses all the liquid inside pipette tip.
        """
        print("Empting tip...")
        speed_ms = 0.5  
        # Moving the robot to the empty tube location
        self.ur.movel(self.empty_tube_above,self.accel_mss,self.speed_ms,0,0)
        sleep(2)
        speed_ms = 0.1
        self.ur.movel(self.empty_tube,self.accel_mss,speed_ms,0,0)
        sleep(2)

        # Drive the pipette three times to dispense all the liquid inside the pipette tip.
        for i in range(3):
            self.dispense_pipette()
            sleep(1)

        self.ur.movel(self.empty_tube_above,self.accel_mss,speed_ms,0,0)
        sleep(1)
    
