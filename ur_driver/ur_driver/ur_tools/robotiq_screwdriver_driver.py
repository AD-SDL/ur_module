from .interpreter_socket import InterpreterSocket
from time import sleep

class RobotiqScrewdriver:
    """
    RobotiqScrewdriver is a class created for UR robots to utilize Robotiq Screwdriver tool.
    This class creates and socket connection utilizing the InterpreterSocket with the UR robot and sends messsages that contains .script commands to control the screwdriver tool.
    """
    def __init__(self, hostname: str = None, port: int = 30020, socket_timeout: float = 2.0) -> None:
        """
        Constructor for the RobotiqScrewdriver class.

        Args:
            hostname (str): The hostname of the robot.
            port (int): Port number to connect to the robot over the Interpreter socket
        """
        self.connection = None
        self.host = hostname

    def connect(self):
        """
        Creates an interpreter socket connection
        """
        try:
            self.connection = InterpreterSocket(ip = self.host)
            self.connection.connect()
        except Exception as err:
            print("Failed to connect to interpreter socket! " + err)
        else: 
            print("Screwdriver connected")
    
    def disconnect(self):
        """
        Closes the Interpreter socket connection
        """
        self.connection.close()
   
    def activate_screwdriver(self):
        """
        Activates the Screwdriver once it is powered on.
        To ensure the consistent activation of the Screwdriver at the start of the program, it is recommended to insert a rq_screw_activate() script function in the BeforeStart sequence of the program.        
        """

        self.connection.execute_command("rq_screw_activate()")

    def activate_vacuum(self):
        """
        Starts the vacuum action on the Screwdriver. This is required for the system to be able to detect a screw.        
        """
        
        self.connection.execute_command("rq_screw_vacuum_on()")

    def deactivate_vacuum(self):
        """
        Stops the vacuum action on the Screwdriver.

        """

        self.connection.execute_command("rq_screw_vacuum_off()")

    def drive_clockwise(self, mode:int = 1, torque: int = 1, angle: int = 360, rpm: int = 100):
        """
        Turns the bit of the Screwdriver in clock wise direction around by a number of degrees or until a specific torque value is reached. 
        Accuracy of the final torque value may vary from standard tolerances.

        mode (int): 1 = RQ_ REGULATION_MODE_ANGLE to turn until a specific number of turns is achieved.
                    3 = RQ_REGULATION_MODE_SPEED to turn until a specific torque value is reached.
        
        torque (int): (unit:Nm) The torque value at which the Screwdriver will stop turning. 
                      Must be between 1 and 4 Nm. Only read when RQ_REGULATION_MODE_SPEED is used.
        
        angle (int): (unit: degrees (°)) The number of degrees by which to turn before the Screwdriver stops. 
                     Only read when RQ_REGULATION_MODE_ANGLE is used.
                     Note: Use “ 0” for rotation_angle in case it is not required.
        
        rpm (int): (unit: RPM) Desired rotation speed of the screwdriver. Must be between 1 and 500 RPM

        """
        direction = True # TRUE = RQ_DIRECTION_CW for a clockwise rotation of the Screwdriver.
        slave_id = 9 # Only slave ID 9 is supported by the Screwdriver.
        
        self.connection.execute_command("rq_screw_turn({},{},{},{},{},{})".format(mode, torque, angle, rpm, direction, slave_id))

    def drive_counter_clockwise(self, mode:int = 1, torque: int = 1, angle: int = 360, rpm: int = 100):
        """
        Turns the bit of the Screwdriver in counter clock wise direction around by a number of degrees or until a specific torque value is reached. 
        Accuracy of the final torque value may vary from standard tolerances.

        mode (int): 1 = RQ_ REGULATION_MODE_ANGLE to turn until a specific number of turns is achieved.
                    3 = RQ_REGULATION_MODE_SPEED to turn until a specific torque value is reached.
        
        torque (int): (unit:Nm) The torque value at which the Screwdriver will stop turning. 
                      Must be between 1 and 4 Nm. Only read when RQ_REGULATION_MODE_SPEED is used.
        
        angle (int): (unit: degrees (°)) The number of degrees by which to turn before the Screwdriver stops. 
                     Only read when RQ_REGULATION_MODE_ANGLE is used.
                     Note: Use “ 0” for rotation_angle in case it is not required.
        
        rpm (int): (unit: RPM) Desired rotation speed of the screwdriver. Must be between 1 and 500 RPM

        """
        direction = False # FALSE= RQ_DIRECTION_CCW for a counter-clockwise rotation of the Screwdriver.
        slave_id = 9 # Only slave ID 9 is supported by the Screwdriver.
        
        self.connection.execute_command("rq_screw_turn({},{},{},{},{},{})".format(mode, torque, angle, rpm, direction, slave_id))
    
    def auto_unscrew(self, rpm: int = 100):
        """
        With the screwdriving bit in or just above the screw drive, performs a full unscrew action until the screw threads are disengaged
        from the hole.

        rpm:    (unit: RPM)
                Desired rotation speed of the screwdriver. Must be between 1 and 500 RPM
        """
        direction = False # FALSE = RQ_DIRECTION_CCW (default) for a counter-clockwise rotation of the Screwdriver
        self.connection.execute_command("rq_auto_unscrew({}, {})".format(direction, rpm))

    
    def auto_screw(self, rpm: int = 100):
        """
        With the screwdriving bit in or just above the screw drive, performs a full screw action until the screw threads are disengaged
        from the hole.

        rpm (int): (unit: RPM) Desired rotation speed of the screwdriver. Must be between 1 and 500 RPM
        """
        direction = True # TRUE = RQ_DIRECTION_CW for a clockwise rotation of the Screwdriver
        self.connection.execute_command("rq_auto_unscrew({}, {})".format(direction, rpm))


if __name__ == "__main__":
    tool = RobotiqScrewdriver("164.54.116.129")  # replace with your device's IP
    tool.connect()
    # tool.auto_unscrew()
    # tool.drive_clockwise()
    # sleep(10)
    # tool.drive_counter_clockwise()
