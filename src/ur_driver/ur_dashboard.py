"""Interface for UR Dashboard"""

import socket
from time import sleep

from paramiko import AutoAddPolicy, SSHClient, SSHException
from scp import SCPClient, SCPException


class UR_DASHBOARD:
    """
    This is a python interface to communicate with the UR Dashboard server.
    The UR can be controlled remotely by sending simple commands to the GUI over a TCP/IP socket.
    The server is running on port 29999 on the robots IP address.
    Each command should be terminated by a `\n` also called a newline.
    """

    def __init__(
        self,
        hostname: str = "164.54.116.129",
        PORT: int = 29999,
    ) -> None:
        """Constructor for the UR dashboard class.
        :param hostname: Hostname or ip.
        :param port: Port.
        """
        self.hostname = hostname
        self.port = PORT
        self.connection = None
        self.connection_error = False
        self.robot_mode = None
        self.safety_status = None
        self.operational_mode = None
        self.remote_control_status = None

        self.connect()
        if self.connection_error:
            raise Exception("Can't connect to dashboard")
        sleep(3)
        self.initialize()

    def connect(self) -> None:
        """Create a socket"""
        try:
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.settimeout(5)  # Socket will wait 5 seconds till it receives the response
            self.connection.connect((self.hostname, self.port))

        except socket.error as err:
            print("UR dashboard could not establish connection")
            print(err)
            self.connection_error = True

    def disconnect(self) -> None:
        """Close the socket"""
        self.connection.close()

    def send_command(
        self,
        command: str = None,
        response_delay: float = 0.1,
    ) -> str:
        """Constructs a command to send over the Dashboard socket
        Args
            command (str): Robot command.
            response_delay (float): Port.
        Return (str): Response message
        """
        try:
            if not self.connection:
                self.connect()

            self.connection.sendall((command.encode("ascii") + b"\n"))

            sleep(response_delay)  # Wait for response delay
            response = self.connection.recv(4096).decode("utf-8")

            if response.find("Connected: Universal Robots Dashboard Server") != -1:
                print("Connected: Universal Robots Dashboard Server")
                response = response[45:]

            print("<< " + response[:-1])

            return response.strip()

        except Exception as err:
            print(err)

    def get_overall_robot_status(self) -> None:
        """Get robot status"""
        self.robot_mode = self.get_robot_mode().upper()
        self.operational_mode = self.get_operational_mode().upper()
        self.safety_status = self.get_safety_status()
        self.remote_control_status = self.is_in_remote_control()

    def initialize(self) -> None:
        """Initializes the robot for Remote operation mode"""
        if self.connection_error:
            return

        self.get_overall_robot_status()

        if self.operational_mode == "MANUAL":
            print("Operation mode is currently set to MANUAL, switching to AUTOMATIC")
            self.set_operational_mode("automatic")

        if self.safety_status == "PROTECTIVE_STOP":
            print("Unlocking protective stop")
            self.unlock_protective_stop()

        elif (
            "NORMAL" not in self.safety_status
        ):  # self.safety_status != "ROBOT_EMERGENCY_STOP" or self.safety_status != "SYSTEM_EMERGENCY_STOP":
            print("Restarting safety")
            self.close_safety_popup()
            self.restart_safety()

        if self.remote_control_status is False:
            print("Robot is not in remote control")

        if self.robot_mode == "RUNNING" and "NORMAL" in self.safety_status:
            print("Robot is initialized")
            return

        elif (
            self.robot_mode == "POWER_OFF"
            or self.robot_mode == "BOOTING"
            or self.robot_mode == "POWER_ON"
            or self.robot_mode == "IDLE"
        ):
            print("Powering on the robot and releasing brakes")
            self.brake_release()

        return self.initialize()

    def get_robot_mode(self) -> str:
        """Return the robot mode
        Return (str): Robot mode
        """
        output = self.send_command("robotmode")
        output = output.split(" ")
        try:
            if "\n" in output[1]:
                return output[1].split("\n")[0]
            else:
                return output[1]
        except IndexError:
            print("Depricated output!")
            return output

    def quit(self) -> str:
        """Closes connection to robot

        Return (str): Socket response
        """
        return self.send_command("quit")

    def shutdown(self) -> str:
        """Shuts down and turns off robot and controller

        Return (str): Socket response
        """
        return self.send_command("shutdown")

    def power_on(self) -> str:
        """Powers on the robot arm

        Return (str): Socket response
        """
        return self.send_command("power on")

    def power_off(self) -> str:
        """Powers off the robot arm

        Return (str): Socket response
        """
        return self.send_command("power off")

    def brake_release(self) -> str:
        """Releases the brakes

        Return (str): Socket response
        """
        output = self.send_command("brake release")
        sleep(20)
        return output

    def unlock_protective_stop(self) -> str:
        """Unlocks the protective stop

        Return (str): Socket response
        """
        return self.send_command("unlock protective stop")

    def close_safety_popup(self) -> str:
        """Closes the safety popup messages

        Return (str): Socket response
        """
        return self.send_command("close safety popup")

    def is_in_remote_control(self) -> str:
        """Checks if robot is in remote control mode

        Return (str): True / False
        """
        print("Remote mode: ")
        return self.send_command("is in remote control")

    def restart_safety(self) -> str:
        """Restarts the safety

        Return (str): Socket response
        """
        output = self.send_command("restart safety")
        sleep(10)
        self.disconnect()
        self.connect()
        output2 = self.brake_release()
        return output + output2

    def get_safety_status(self) -> str:
        """Returns safety status

        Return (str): Safety status
        """
        output = self.send_command("safetystatus")
        output = output.split(" ")
        # print(output)
        try:
            if "\n" in output[1]:
                return output[1].split("\n")[0]
            else:
                return output[1]
        except IndexError:
            print("Depricated output!")
            return output

    def get_operational_mode(self) -> str:
        """Gets operational mode

        Return (str): Operational mode (Manual/Auto)
        """
        return self.send_command("get operational mode")

    def set_operational_mode(self, mode) -> str:
        """Sets the operational mode

        Args: mode(str) Either manual or auto

        Return (str): Socket response
        """
        return self.send_command("set operational mode " + mode)

    def clear_operational_mode(self) -> str:
        """Clears the operational mode when the control is stuck in one mode

        Return (str): Socket response
        """
        return self.send_command("clear operational mode")

    def popup(self, message) -> str:
        """Display a custom popup message

        Args: message (str) Costom message

        Return (str): Socket response
        """
        return self.send_command("popup " + message)

    def close_popup(self) -> str:
        """Closes the popup messages

        Return (str): Socket response
        """
        return self.send_command("close popup")

    def transfer_program(
        self,
        local_path: str = None,
        remote_path: str = "/programs/",
        user_name: str = "root",
        user_password: str = "easybot",
    ) -> None:
        """Trasnfers a URP program from local path to Robot computer

        Args
            local_path (str): Local path to URP program
            remote_path (str): Remote path on UR computer
            user_name (str): User name for UR computer
            user_password(str): User password for UR compurter
        """
        if not local_path:
            print("Local file was not provided!")
            return
        try:
            ssh_client = SSHClient()
            ssh_client.load_system_host_keys()
            ssh_client.set_missing_host_key_policy(AutoAddPolicy())
            ssh_client.connect(
                hostname=self.hostname,
                username=user_name,
                password=user_password,
                disabled_algorithms={
                    "pubkeys": [
                        "rsa-sha2-256",
                        "rsa-sha2-512",
                    ]
                },
            )
            with SCPClient(ssh_client.get_transport()) as scp:
                scp.put(local_path, remote_path)

        except SSHException as scp_err:
            print("SSH error: " + scp_err)

        except SCPException as scp_err:
            print("SCP error: " + scp_err)

        else:
            print("UR program " + local_path + " is transferred to UR onboard " + remote_path)

        finally:
            scp.close()
            ssh_client.close()

    def load_program(self, program_path: str) -> str:
        """Load a URP program on Polyscope

        Args: program_path (str) Path to the URP program on UR computer

        Return (str): Socket response
        """
        return self.send_command("load " + program_path)

    def get_program_state(self) -> str:
        """Gets the currently loaded program state

        Return (str): Program state (Running/Stopped/Error)
        """
        return self.send_command("programState")

    def get_loaded_program(self) -> str:
        """Gets the currently loaded program name

        Return (str): Program name
        """
        return self.send_command("get loaded program")

    def get_program_run_status(self) -> str:
        """Gets program run status

        Return (str): Status
        """
        return self.send_command("running")

    def run_program(self) -> str:
        """Runs the loaded program

        Return (str): Socket response
        """
        return self.send_command("play")

    def pause_program(self) -> str:
        """Pauses the currently running program

        Return (str): Socket response
        """
        return self.send_command("pause")

    def stop_program(self) -> str:
        """Stops the currently running program

        Return (str): Socket response
        """
        return self.send_command("stop")


if __name__ == "__main__":
    """Tests"""
    robot = UR_DASHBOARD("164.54.116.129")
    # robot.get_loaded_program()
    # robot.get_program_state()
    # a = robot.load_program("/programs/interpreter_mode.urp")
    # if "File not found" in a:
    #     print(a)
    # robot.get_overall_robot_status()
    # robot.get_operational_mode()
    # robot.robot_mode()
    # robot.close_popup()
    # robot.initialize()
    # robot.get_program_run_status()
    # robot.load_program("/home/rpl/test.txt")
    # robot.get_loaded_program()
    # robot.power_on()
    # robot.brake_release()
    # robot.power_off()
    # robot.brake_release()
    # robot.self.get_safety_status()
    # robot.quit()
    # robot.clear_operational_mode()
    # robot.transfer_program("/home/rpl/test.urp", "/programs/katerina.urp")
    # robot.load_program("/programs/katerina.urp")
    # robot.run_program()
