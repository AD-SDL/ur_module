"""OT Pipette Controller Interface"""

from time import sleep
from abstract_pipette_interfaces import Pipette

import epics


class EpicsOpentronsPipette(Pipette):
    """A class to control the OT pipette over tbe EPICS PVs"""

    def __init__(self, pipette_pv: str = None):
        """
        Initializes the OTPipetteController class to controll the OpenTron pipettes.

        Parameters:
        - pipette_pv (str): The EPICS process variable (PV) for the pipette.
        - ur_connection: The connection object for the Universal Robot (UR) robot.
        """

        self.pv = pipette_pv
        self.connect_pipette()

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

    def aspirate(self, volume: float):
        """
        Description:
            - Drives pipette to aspirate liquid.
            - Number of motor steps to aspirate liquid is stored in "self.pipette_aspirate_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Aspirating the sample...")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) + volume)
        sleep(1)

    def dispense(self, volume: float):
        """
        Description:
            - Drives pipette to dispense liquid.
            - Number of motor steps to dispense liquid is stored in "self.pipette_dispense_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Dispensing sample")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) + volume)
        sleep(1)

    def create_droplet(self, volume: float):
        """
        Description:
            - Drives pipette to create a droplet.
            - Number of motor steps to create a droplet is stored in "self.droplet_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Creating a droplet...")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) - volume)
        sleep(10)

    def retrieve_droplet(self, volume: float):
        """
        Description:
            - Retrieves the droplet back into the pipette tip.
            - Number of motor steps to retrieve a droplet is stored in "self.droplet_value".
            - Pipette is controlled by pyepics PV commands.
        """
        print("Retrieving droplet...")
        current_value = self.pipette.get()
        self.pipette.put(float(current_value) + volume + 0.5)
        sleep(1)
