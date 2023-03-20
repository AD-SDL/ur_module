import socket
from time import sleep

class UR_DASHBOARD():
    def __init__(self, IP:str = "146.137.240.38", PORT: int = 29999):

        self.IP = IP
        self.port = PORT
        self.connection = None

        self.connect()

    def connect(self):
        """Create a socket"""
        try:
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.settimeout(5) # Socket will wait 10 seconds till it recieves the response
            self.connection.connect((self.IP,self.port))

        except Exception as err:
            print("UR dashboard could not establish connection")
            print(err)

    def disconnect(self):
        """Close the socket"""
        self.connection.close()

    def send_command(self, command):

        print(">> " + command)

        try:

            if not self.connection:
                self.connect()

            self.connection.sendall((command.encode("ascii") + b"\n")) #Check these to see if respond was received properly
            sleep(2)
            response = self.connection.recv(4096).decode("utf-8")
                
            print("<< " + response)
            return response

        except Exception as err:
            print(err)

    def initialize(self):

        robot_mode = self.robot_mode()
        operation_mode = self.get_operational_mode()
        safety_status = self.safety_status()
        remote_control_status = self.is_in_remote_control()
        popup = self.popup()
        print("AAAA:", robot_mode)
        if robot_mode.upper() == "POWER OFF":
            print("Powering on the robot")
            self.brake_release()
    

        if operation_mode.upper() == "MANUAL":
            print("Operation mode is currently set to MANUAL, switching to AUTOMATIC")
            self.set_operational_mode("automatic")

        if remote_control_status == False:
            print("Robot is not in remote control")

        if safety_status.upper() != "NORMAL" or safety_status.upper() != "ROBOT_EMERGENCY_STOP" or safety_status.upper() != "SYSTEM_EMERGENCY_STOP":
            print(safety_status)
            print("Resarting safety")
            self.close_safety_popup()
            # self.restart_safety()

        if popup != None:
            self.close_popup()


    def robot_mode(self):
        """Return the robot mode"""
        #TODO: Can be used to check if robot joint are moving
        return self.send_command("robotmode")
        
    def quit(self):
        '''Closes connection to robot'''
        return self.send_command('quit')

    def shutdown(self):
        '''Shuts down and turns off robot and controller'''
        return self.send_command('shutdown')

    def power_on(self):
        '''Powers on the robot arm'''
        return self.send_command('power on')

    def power_off(self):
        '''Powers off the robot arm'''
        return self.send_command('power off')

    def brake_release(self):
        '''Releases the brakes'''
        return self.send_command('brake release')

    def unlock_protective_stop(self):
        return self.send_command('unlock protective stop')

    def close_safety_popup(self):
        return self.send_command('close safety popup')

    def is_in_remote_control(self):
        return self.send_command('is in remote control')

    def restart_safety(self):
        return self.send_command('restart safety')
        
    def safety_status(self):
        return self.send_command('safetystatus')

    def get_operational_mode(self):
        return self.send_command('get operational mode')

    def set_operational_mode(self):
        return self.send_command('set operational mode')

    def popup(self):
        return self.send_command('popup')

    def close_popup(self):
        return self.send_command('close popup')

if __name__ == "__main__":
    robot = UR_DASHBOARD()
    robot.power_on()
    # robot.initialize()
    robot.brake_release()
    # robot.power_off()
    # robot.brake_release()