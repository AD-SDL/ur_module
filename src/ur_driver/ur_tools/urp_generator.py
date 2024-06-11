"""URP Generator Class"""


class URPGenerator:
    """This object is designed to genereate URP scripts on local to be later trasnfer on to the Polyscope"""

    def __init__(self, filename: str = None):
        """Contractor for the URPGenerator

        Args:
            filename (str): Filename to be used after the URP program is generated
        """
        self.filename = filename
        self.program = ""

    def add_line(self, line: str = None):
        """Adds a new line

        Args:
            line (str): A  string to be added as a new line
        """
        self.program += line + "\n"

    def add_activate_tool(self, tool_name: str = None):
        """Adds the necessary lines to activate a tool into the URP program

        Args:
            tool_name (str): Name of the tool
        """
        self.add_line("def activate_tool():")
        self.add_line("    textmsg('Activating Robotiq screwdriver tool')")
        self.add_line("")
        self.add_line("    # Set tool voltage")
        self.add_line("    set_tool_voltage(24)")
        self.add_line("")
        self.add_line("    # Set tool digital output to activate screwdriver")
        self.add_line("    set_tool_digital_out(0, True)")
        self.add_line("")
        self.add_line("    # Wait for activation")
        self.add_line("    sleep(2)")  # Adjust the sleep duration as needed
        self.add_line("")
        self.add_line("    textmsg('Robotiq screwdriver tool activated')")
        self.add_line("")
        self.add_line("end")

    def add_pick_and_place(
        self,
        pick_location: str = None,
        place_location: str = None,
    ):
        """Adds the necessary lines for pick and place jobs into the URP program

        Args:
            pick_location (str): Pick location
            place_location (str): Place location
        """
        self.add_line("def pick_and_place():")
        self.add_line("    textmsg('Starting pick and place operation')")
        self.add_line("")
        self.add_line("    # Move to pick location")
        self.add_line(f"    movel(p{pick_location}, a=0.5, v=0.5)")
        self.add_line("    textmsg('Reached pick location')")
        self.add_line("")
        self.add_line("    # Perform pick action")
        self.add_line("    # Add your pick action here")
        self.add_line("")
        self.add_line("    # Move to place location")
        self.add_line(f"    movel(p{place_location}, a=0.5, v=0.5)")
        self.add_line("    textmsg('Reached place location')")
        self.add_line("")
        self.add_line("    # Perform place action")
        self.add_line("    # Add your place action here")
        self.add_line("")
        self.add_line("    textmsg('Pick and place operation completed')")
        self.add_line("")
        self.add_line("end")

    def add_drive_screw(
        self,
        torque: float = None,
        rotation_speed: float = None,
    ):
        """Adds the necessary lines for driving the screwdriver into the URP program

        Args:
            torque (float): NM
            rotation_speed (float): RPM
        """
        self.add_line("def drive_screw(torque, rotation_speed):")
        self.add_line("    textmsg('Driving screw')")
        self.add_line("")
        self.add_line("    # Set screwdriving torque")
        self.add_line("    set_tool_digital_out(1, True)")
        self.add_line("    set_analog_out(0, torque)")
        self.add_line("")
        self.add_line("    # Rotate screwdriver at specified speed")
        self.add_line("    set_analog_out(1, rotation_speed)")
        self.add_line("")
        self.add_line("    # Wait for screwdriving")
        self.add_line("    sleep(2)")  # Adjust the sleep duration as needed
        self.add_line("")
        self.add_line("    # Stop rotating screwdriver")
        self.add_line("    set_analog_out(1, 0)")
        self.add_line("")
        self.add_line("    textmsg('Screwdriving completed')")
        self.add_line("")
        self.add_line("end")

    def add_deactivate_tool(self):
        """Adds the necessary lines to deactivate the tool into the URP program"""
        self.add_line("def deactivate_tool():")
        self.add_line("    textmsg('Deactivating Robotiq screwdriver tool')")
        self.add_line("")
        self.add_line("    # Set tool digital outputs to deactivate screwdriver")
        self.add_line("    set_tool_digital_out(0, False)")
        self.add_line("    set_tool_digital_out(1, False)")
        self.add_line("")
        self.add_line("    # Set analog outputs to zero")
        self.add_line("    set_analog_out(0, 0)")
        self.add_line("    set_analog_out(1, 0)")
        self.add_line("")
        self.add_line("    textmsg('Robotiq screwdriver tool deactivated')")
        self.add_line("")
        self.add_line("end")

    def generate_program(self):
        """Gerates the program and saves it to local path"""
        with open(self.filename, "w") as file:
            file.write(self.program)

        print(f".urp program '{self.filename}' generated successfully.")


# Usage example
pick_location = [
    0.1,
    0.2,
    0.3,
    0,
    3.14159,
    0,
]  # Example pick location (x, y, z, rx, ry, rz)
place_location = [
    0.4,
    0.5,
    0.6,
    0,
    3.14159,
    0,
]  # Example place location (x, y, z, rx, ry, rz)

# generator = URPGenerator("pick_and_place.urp")
# generator.add_pick_and_place(pick_location, place_location)
# generator.generate_program()
