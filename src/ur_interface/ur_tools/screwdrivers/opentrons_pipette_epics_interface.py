"""Robotiq Screwdriver Driver Class"""


"""Interpreter Socket to create remote communication with Interpreter URP generator that runs on UR polyscope"""

import logging
import re
import socket
from time import sleep
from abstract_screwdriver_interfaces import Screwdriver, Direction

UR_INTERPRETER_SOCKET = 30020


class InterpreterSocket:
    """This socket creates a remote interface with the interpreter urp program generator to create urp programs on the polyscope remotely"""

    log = logging.getLogger("interpreter.InterpreterHelper")
    STATE_REPLY_PATTERN = re.compile(r"(\w+):\W+(\d+)?")

    def __init__(
        self,
        hostname: str = None,
        port: int = UR_INTERPRETER_SOCKET,
        timeout: float = 2.0,
    ) -> None:
        """Constructor for the InterpreterSocket class.
        :param hostname: Hostname or ip.
        :param port: Port.
        :param socket_timeout: Timeout for blocking socket operations.
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.hostname = hostname
        self.port = port
        self.timeout = timeout
        self.response = None

    def connect(self) -> None:
        """
        Connects to a tool at the given address.
        """
        try:
            self.socket.connect((self.hostname, self.port))
            self.socket.settimeout(self.timeout)
        except socket.error as exc:
            self.log.error(f"Interpreter Socket Error = {exc}")
            raise exc

    def disconnect(self) -> None:
        """Closes the connection with the Interpreter socket."""
        self.socket.close()

    def get_reply(self):
        """
        read one line from the socket
        :return: text until new line
        """
        collected = b""
        while True:
            part = self.socket.recv(4096)
            if part != b"\n":
                collected += part
            elif part == b"\n":
                break
        self.response = collected.decode("utf-8")

    def execute_command(self, command) -> int:
        """
        Send single line command to interpreter mode, and wait for reply
        :param command:
        :return: ack, or status id
        """
        self.log.debug(f"Command: '{command}'")
        if not command.endswith("\n"):
            command += "\n"

        self.socket.send(command.encode("utf-8"))
        self.get_reply()
        self.log.debug(f"Reply: '{self.response}'")
        # parse reply, raise exception if command is discarded
        reply = self.STATE_REPLY_PATTERN.match(self.response)
        if reply.group(1) == "discard":
            raise Exception(
                "Interpreter discarded message",
                self.response,
            )
        return int(reply.group(2))

    def clear(self):
        """Clear the urp programs"""
        return self.execute_command("clear_interpreter()")

    def skip(self):
        """Skips the buffer"""
        return self.execute_command("skipbuffer")

    def abort_move(self):
        """Abort"""
        return self.execute_command("abort")

    def get_last_interpreted_id(self):
        """Gets the last interpreted ID"""
        return self.execute_command("statelastinterpreted")

    def get_last_executed_id(self):
        """Gets the last executed ID"""
        return self.execute_command("statelastexecuted")

    def get_last_cleared_id(self):
        """Get the last cleared ID"""
        return self.execute_command("statelastcleared")

    def get_unexecuted_count(self):
        """Get unexecuted line count"""
        return self.execute_command("stateunexecuted")

    def end_interpreter(self):
        """End interpreter program"""
        return self.execute_command("end_interpreter()")

class RobotiqScrewdriver(Screwdriver):
    """
    RobotiqScrewdriver is a class created for UR robots to utilize Robotiq Screwdriver tool.
    This class creates and socket connection utilizing the InterpreterSocket with the UR robot and sends messsages that contains .script commands to control the screwdriver tool.
    """

    def __init__(
        self,
        hostname: str = None,
        port: int = 30020,
        socket_timeout: float = 2.0,
    ) -> None:
        """
        Constructor for the RobotiqScrewdriver class.

        Args:
            hostname (str): The hostname of the robot.
            port (int): Port number to connect to the robot over the Interpreter socket
        """
        self.connection = None
        self.hostname = hostname
        self.timeout = socket_timeout

    def connect(self) -> None:
        """
        Creates an interpreter socket connection
        """
        try:
            self.connection = InterpreterSocket(hostname=self.hostname, timeout=self.timeout)
            self.connection.connect()
        except Exception as err:
            print("Failed to connect to interpreter socket! " + err)
        else:
            print("Screwdriver connected")

    def disconnect(self) -> None:
        """
        Closes the Interpreter socket connection
        """
        self.connection.disconnect()

    def get_status(self) -> str:
        """
        Retrieves the current activation status of the Screwdriver.

        Return (int): Status
                        - 3 = Activated
                        - 2 = Not used
                        - 1 = Activation in progress
                        - 0 = Else
        """
        self.connection.execute_command("rq_get_screw_status()")
        return self.connection.response

    def get_vacuum_pressure_kpa(self) -> str:
        """
        Retrieves the current vacuum level in the vacuum sleeve.

        Return (int): Value = Current vacuum level in the vacuum sleeve in kPa.
                              Without screw vacuum must be equal or superior to -16 kPa
                              With screw vWacuum must be equal or inferior to -30 kPa
        """
        self.connection.execute_command("rq_get_vacuum_pressure_kpa()")
        return self.connection.response

    def is_blocked_at_start(self) -> str:
        """
        The Screwdriver started fastening a screw on which torque has already been applied.
        Use this script under the “ Unhandled errors” node to display the cause of the error.

        Return (bool): TRUE = The cause of the “ Unhandled errors” is “ Blocked at start” .
        """
        self.connection.execute_command("rq_is_blocked_at_start_cause()")
        return self.connection.response

    def is_communication_lost(self) -> str:
        """
        The communication between the Screwdriver and the robot controller has been lost.
        Use this script under the “ Unhandled errors” node to display the cause of the error.

        Return (bool): TRUE = The cause of the “ Unhandled errors” is “ Communication lost” .

        """
        self.connection.execute_command("rq_is_communication_lost_cause()")
        return self.connection.response

    def is_contact_not_found(self) -> str:
        """
        The Screwdriver did not find contact with the surface.
        Use this script under the “ Unhandled errors” node to display the cause of the error.

        Return (bool): TRUE = The cause of the “ Unhandled errors” is “ Contact not found” .
        """
        self.connection.execute_command("rq_is_contact_not_found_cause()")
        return self.connection.response

    def is_feeder_screw_ready(self, inputNumber: int = 0) -> str:
        """
        Validates if a screw is ready to be picked at the screw detection position of the Screw Feeder.

        Args: inputNumber (int): Integer associated with the “ Screw ready” digital input.

        Return (str): TRUE = A screw is detected by the screw detection sensor of the Screw Feeder.

        """
        self.connection.execute_command("rq_is_feeder_screw_ready({})".format(inputNumber))
        return self.connection.response

    def is_feeder_status_ok(self, inputNumber: int = 1) -> str:
        """
        Validates if the Screw Feeder is on and ready.

        Args: inputNumber (int): Integer associated with the digital input of “ Feeder status” .

        Return (str): TRUE = The Screw Feeder is turned on and detects no errors.

        """
        self.connection.execute_command("rq_is_feeder_status_ok({})".format(inputNumber))
        return self.connection.response

    def is_restricted_before_angle(self) -> str:
        """
        The torque value defined at the teaching step has been reached earlier than expected.
        Use this script under the “ Unhandled errors” node to display the cause of the error.

        Return (str) TRUE = The cause of the “ Unhandled errors” is “ Restriction before angle” .
        """
        self.connection.execute_command("rq_is_restricted_before_angle_cause()")
        return self.connection.response

    def is_screw_detected(self) -> str:
        """
        Validates the presence of a screw at the end of the screwdriver via vacuum action. The vacuum needs to be turned on for the system to be able to detect the screw.
        Return (str): TRUE = A screw is detected at the end of the vacuum sleeve.
        """
        self.connection.execute_command("rq_is_screw_detected()")
        return self.connection.response

    def is_screwdriver_wrong_echo(self) -> str:
        """
        There is a discrepancy between the command sent to the Screwdriver and the action performed by the Screwdriver. Cross threading may have caused this issue.
        Use this script under the “ Unhandled errors” node to display the cause of the error.

        Return (str): TRUE = The cause of the “ Unhandled errors“ is “ Wrong echo” .
        """
        self.connection.execute_command("rq_is_screwdriver_wrong_echo_cause()")
        return self.connection.response

    def pick_screw_from_feeder(
        self,
        screw_pose: str = None,
        status_input: int = 1,
        ready_input: int = 0,
        test_in_action: bool = False,
        approach_dist: int = 1,
        rpm: int = 100,
        pick_speed: int = 1,
        timeout: int = 10,
        retract_speed: int = 1,
        direction: bool = False,
        slave_id: int = 9,
    ) -> None:
        """
        Picks a screw from the Robotiq Screw Feeder.
        Args:
            screw_pose (str): Position where the robot will be picking the screw. Defined in Installation under Features.

            status_input (int): Digital input used for “ Feeder status” input.

            ready_input (int): Digital input used for “ Screw ready” input.

            test_in_action (bool): TRUE = The robot stops after picking a screw.
                                   FALSE = The robot moves on to the next line in the program after picking a screw.

            approach_dist (int): (unit: mm) Distance between the approach point and the screw_pose point

            rpm (int): (unit: RPM) Desired rotation speed of the screwdriver. Must be between 1 and 500 RPM

            pick_speed (int): (unit: mm/s) Movement speed between the approach point and the screw_pose point.

            timeout (int): (unit: sec) Time after which the robot will stop trying to pick a screw if the previous pick was unsuccessful.

            retract_speed (int): (unit: mm/s) Movement speed between the screw_pose point and the retract point.

            direction (bool): FALSE = RQ_DIRECTION_CCW (default) for a counter-clockwise rotation of the Screwdriver.
                              TRUE = RQ_DIRECTION_CW for a clockwise rotation of the Screwdriver.

            slave_id (int): Only slave ID 9 is supported by the Screwdriver.
        """
        self.connection.execute_command(
            "rq_pick_screw({},{},{},{},{},{},{},{},{},{},{})".format(
                screw_pose,
                status_input,
                ready_input,
                test_in_action,
                approach_dist,
                rpm,
                pick_speed,
                timeout,
                retract_speed,
                direction,
                slave_id,
            )
        )

    def activate_screwdriver(self) -> None:
        """
        Activates the Screwdriver once it is powered on.
        To ensure the consistent activation of the Screwdriver at the start of the program, it is recommended to insert a rq_screw_activate() script function in the BeforeStart sequence of the program.
        """
        self.connection.execute_command("rq_screw_activate()")

    def stop_screwdriver(self) -> None:
        """
        Stops the rotation of the screwdriver.
        """
        self.connection.execute_command("rq_screw_stop()")

    def activate_vacuum(self) -> None:
        """
        Starts the vacuum action on the Screwdriver. This is required for the system to be able to detect a screw.
        """
        self.connection.execute_command("rq_screw_vacuum_on()")

    def deactivate_vacuum(self) -> None:
        """
        Stops the vacuum action on the Screwdriver.
        """
        self.connection.execute_command("rq_screw_vacuum_off()")

    def drive_clockwise(
        self,
        mode: int = 1,
        torque: int = 1,
        angle: int = 360,
        rpm: int = 100,
    ) -> None:
        """
        Turns the bit of the Screwdriver in clock wise direction around by a number of degrees or until a specific torque value is reached.
        Accuracy of the final torque value may vary from standard tolerances.

        Args:
            mode (int): 1 = RQ_ REGULATION_MODE_ANGLE to turn until a specific number of turns is achieved.
                        3 = RQ_REGULATION_MODE_SPEED to turn until a specific torque value is reached.

            torque (int): (unit:Nm) The torque value at which the Screwdriver will stop turning.
                        Must be between 1 and 4 Nm. Only read when RQ_REGULATION_MODE_SPEED is used.

            angle (int): (unit: degrees (°)) The number of degrees by which to turn before the Screwdriver stops.
                        Only read when RQ_REGULATION_MODE_ANGLE is used.
                        Note: Use “ 0” for rotation_angle in case it is not required.

            rpm (int): (unit: RPM) Desired rotation speed of the screwdriver. Must be between 1 and 500 RPM

        """
        direction = True  # TRUE = RQ_DIRECTION_CW for a clockwise rotation of the Screwdriver.
        slave_id = 9  # Only slave ID 9 is supported by the Screwdriver.

        self.connection.execute_command(
            "rq_screw_turn({},{},{},{},{},{})".format(
                mode,
                torque,
                angle,
                rpm,
                direction,
                slave_id,
            )
        )

    def drive_counter_clockwise(
        self,
        mode: int = 1,
        torque: int = 1,
        angle: int = 360,
        rpm: int = 100,
    ) -> None:
        """
        Turns the bit of the Screwdriver in counter clock wise direction around by a number of degrees or until a specific torque value is reached.
        Accuracy of the final torque value may vary from standard tolerances.

        Args:
            mode (int): 1 = RQ_ REGULATION_MODE_ANGLE to turn until a specific number of turns is achieved.
                        3 = RQ_REGULATION_MODE_SPEED to turn until a specific torque value is reached.

            torque (int): (unit:Nm) The torque value at which the Screwdriver will stop turning.
                        Must be between 1 and 4 Nm. Only read when RQ_REGULATION_MODE_SPEED is used.

            angle (int): (unit: degrees (°)) The number of degrees by which to turn before the Screwdriver stops.
                        Only read when RQ_REGULATION_MODE_ANGLE is used.
                        Note: Use “ 0” for rotation_angle in case it is not required.

            rpm (int): (unit: RPM) Desired rotation speed of the screwdriver. Must be between 1 and 500 RPM

        """
        direction = False  # FALSE= RQ_DIRECTION_CCW for a counter-clockwise rotation of the Screwdriver.
        slave_id = 9  # Only slave ID 9 is supported by the Screwdriver.

        self.connection.execute_command(
            "rq_screw_turn({},{},{},{},{},{})".format(
                mode,
                torque,
                angle,
                rpm,
                direction,
                slave_id,
            )
        )

    def auto_unscrew(self, rpm: int = 100) -> None:
        """
        With the screwdriving bit in or just above the screw drive, performs a full unscrew action until the screw threads are disengaged
        from the hole.

        Args:
            rpm:    (unit: RPM)
                    Desired rotation speed of the screwdriver. Must be between 1 and 500 RPM
        """
        direction = False  # FALSE = RQ_DIRECTION_CCW (default) for a counter-clockwise rotation of the Screwdriver
        self.connection.execute_command("rq_auto_unscrew({}, {})".format(direction, rpm))

    def auto_screw(self, rpm: int = 100) -> None:
        """
        With the screwdriving bit in or just above the screw drive, performs a full screw action until the screw threads are disengaged
        from the hole.

        Args:
            rpm (int): (unit: RPM) Desired rotation speed of the screwdriver. Must be between 1 and 500 RPM
        """
        direction = True  # TRUE = RQ_DIRECTION_CW for a clockwise rotation of the Screwdriver
        self.connection.execute_command("rq_auto_unscrew({}, {})".format(direction, rpm))
    

    def turn_screwdriver(self, direction, duration = 120):
        if direction == Direction.CLOCKWISE:
            self.drive_clockwise(mode=3, torque=4, angle=0, rpm=100)
            sleep(duration)
            self.stop_screwdriver()
        elif direction == Direction.COUNTERCLOCKWISE:
            self.drive_counter_clockwise(mode=3, torque=4, angle=0, rpm=100)
            sleep(duration)
            self.stop_screwdriver()

