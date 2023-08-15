from time import sleep
from copy import deepcopy


class ApsPipetteController():

    def __init__(self, ur_connection = None):
        """
        Initializes the PipetteController class.
        
        Parameters:
        - pipette_pv (str): The EPICS process variable (PV) for the pipette.
        - ur_connection: The connection object for the Universal Robot (UR) robot.
        """



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

        # self.pipette_loc = [-0.30710397664568057, 0.2223363316295067, 0.25346649921490616, 0.9780579194931717, -1.3456500374612195, -1.5122814896417478]
        # self.pipette_loc_J = [2.8711442947387695, -1.8251310787596644, -1.5156354904174805, -1.3721376222423096, -4.720762554799215, -3.0886977354632776]
        # self.pipette_approach = [-0.30710468347240427, 0.22234393663902577, 0.2793166289891617, 0.977973715529265, -1.3455795650125528, -1.512392593568845]
        # self.pipette_approach_J = [2.8711442947387695, -1.8102451763548792, -1.412275791168213, -1.4902585309794922, -4.720990244542257, -3.088721577321188]
        # self.pipette_above = [-0.3075719688934094, 0.2227307810713913, 0.40515454739546075, 0.9815940238325527, -1.3416684284127856, -1.504904936327573]
        # self.pipette_above_J = [2.869802236557007, -1.9749981365599574, -0.5613865852355957, -2.1772977314391078, -4.720307175313131, -3.0981438795672815]
        # self.tip1_loc = [0.049076405377552826, 0.35130426249264163, 0.063, 0.9759108742683295, -1.3350220046082053, -1.5092226077826993]
        # self.tip1_approach = [0.049076095756735466, 0.3513032390285145, 0.083, 0.9758916159413838, -1.3350252553821587, -1.5092057412143818]
        # self.tip1_above = [0.04908221782054774, 0.3513003341332178, 0.138, 0.9758574103691817, -1.3350463108315163, -1.5091909291569083]
        # self.tip2_loc = [0.04909177440821851, 0.3411316353820866, 0.0628, 0.977119433532159, -1.3337829736507698, -1.5108373189678133]
        # self.tip2_approach = [0.04909177440821851, 0.3411316353820866, 0.083, 0.977119433532159, -1.3337829736507698, -1.5108373189678133]
        # self.tip2_above = [0.04909177440821851, 0.3411316353820866, 0.138, 0.977119433532159, -1.3337829736507698, -1.5108373189678133]
        # self.sample1 = [0.15220619381604186, 0.21043816573205595, 0.09618091909170277, 1.444826407763332, -0.2548060433102738, -0.31289353129621067]
        # self.sample1_above = [0.15220723461648447, 0.2104311001071656, 0.14402782259610025, 1.4448359749910735, -0.2548206714588542, -0.31295915781137074]
        # self.sample2_above = [0.15279755520703087, 0.18939793717407497, 0.14402267332894347, 1.444821393022025, -0.25485812796155616, -0.3128929822914916]
        # self.sample2 = [0.15186061464767017, 0.18822197623964088, 0.09490910394912143, 1.4440966224799245, -0.255613147568461, -0.3122426586441542]
        # self.empty_tube = [0.15203368788019977, 0.16531582069324421, 0.12185568609417977, 1.4402850302548993, -0.2846256403901101, -0.3468228184833902]
        # self.empty_tube_above = [0.15203001904780783, 0.16531236663764431, 0.14222620538915642, 1.4402337440190125, -0.2846450307479814, -0.346876615018759]
        # self.well1 = [0.12772478460859046, 0.21370236710062357, 0.08390608100945282, 1.4380130231592743, -0.2414629895555231, -0.2954608172533908]
        # self.well1_above = [0.12773445855037924, 0.21371308008717516, 0.1271232135439438, 1.4380596200664426, -0.24151536289689018, -0.2954919320386042]
        # self.trash_bin_above = [0.187412530306272, 0.2868009561100828, 0.12712991727750073, 1.438076830279249, -0.2414934112798892, -0.2954944172453427]
        # self.trash_bin = [0.1874179391982658, 0.2867862635600429, 0.013156853887081085, 1.438022625162957, -0.24148065729851562, -0.2954808450568972]

    def connect_pipette(self):
        """
        Connect pipette
        """

        try:
            # Establishing a connection with the pipette on EPICS
            self.pipette = epics.PV(self.pv)
            
        except Exception as err:
            print("Pipette error: ", err)

        else:
            print("Pipette is connected")
        pass

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

    def move_pipette_dock(self):
        """
        Description: Moves the robot to the doscking location and then picks up the pipette.
        """
        print("Picking up the pipette...")
        accel_mss = 1.00
        speed_ms = 1.00
    
        print("Picking up the pipette...")
        sleep(1)
        self.ur.movel(self.pipette_above,self.accel_mss,speed_ms,0,0)
        sleep(2)
        self.ur.movel(self.pipette_approach,self.accel_mss,speed_ms,0,0)
        speed_ms = 0.01
        sleep(1)
        self.ur.movel(self.pipette_loc,self.accel_mss,speed_ms,0,0)
        sleep(5)
        # LOCK THE TOOL CHANGER TO ATTACH THE PIPETTE HERE

    def lift_pipette_on_dock(self):
        """
        Description: Moves the robot to the doscking location and then picks up the pipette.
        """
        sleep(5.0)
        self.ur.movel(self.pipette_approach,self.accel_mss,speed_ms,0,0)
        sleep(1)
        speed_ms = 0.1
        self.ur.movel(self.pipette_above,self.accel_mss,speed_ms,0,0)
        sleep(2)
        print("Pipette successfully picked up")

    def pick_tip(self, tip_loc, x=0, y=0):
        """
        Description: Picks up a new tip from the first location on the pipette bin.
        """

        tip_approach = deepcopy(tip_loc)
        tip_approach[2] += 0.02
        tip_above = deepcopy(tip_loc)
        tip_above[2] += 0.075

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

    def transfer_sample(self, sample_loc, well_loc):
        
        """
        Description: 
            - Makes a new sample on the 96 well plate.
            - Mixes to liquits in a single well and uses a new pipette tip for each liquid.
            - In order to mix the liquids together, pipette performs aspirate and dispense operation multiple times in the well that contains both the liquids.
        """
        print("Making a sample using two liquids...")
        
        # MOVE TO THE FIRT SAMPLE LOCATION
        speed_ms = 0.1

        sample_above = sample_loc
        sample_above[2] += 0.05
        well_above = well_loc
        well_above[2] += 0.05

        self.ur.movel(sample_above,self.accel_mss,self.speed_ms,0,0)
        sleep(2)
        self.ur.movel(sample_loc,self.accel_mss,speed_ms,0,0)
        sleep(2)

        # ASPIRATE FIRST SAMPLE
        self.aspirate_pipette()
        self.ur.movel(sample_above,self.accel_mss,speed_ms,0,0)
        sleep(1)

        # MOVE TO THE 1ST WELL
        self.ur.movel(well_above,self.accel_mss,speed_ms,0,0)
        sleep(1)
        self.ur.movel(well_loc,self.accel_mss,speed_ms,0,0)
        sleep(1)

        # DISPENSE FIRST SAMPLE INTO FIRST WELL
        self.dispense_pipette()
        self.ur.movel(well_above,self.accel_mss,speed_ms,0,0)
        sleep(1)


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

    def dispense_pipette(self):
        """
        Description: 
            - Drives pipette to dispense liquid. 
            - Number of motor steps to dispense liquid is stored in "self.pipette_dispense_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Dispensing sample")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value)+ self.pipette_dispense_value)
        sleep(1)

    def create_droplet(self):
        """
        Description: 
            - Drives pipette to create a droplet.
            - Number of motor steps to create a droplet is stored in "self.droplet_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Creating a droplet...")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) - self.droplet_value)
        sleep(10)
  

    def retrieve_droplet(self):
        """
        Description: 
            - Retrieves the droplet back into the pipette tip.
            - Number of motor steps to retrieve a droplet is stored in "self.droplet_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Retrieving droplet...")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) + self.droplet_value + 0.5)
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
    
