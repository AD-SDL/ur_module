import socket
from time import sleep

from paramiko import SSHClient, AutoAddPolicy, SSHException
from scp import SCPClient, SCPException

class UR_DASHBOARD():
    def __init__(self, hostname:str = "146.137.240.38", PORT: int = 29999):

        self.hostname = hostname
        self.port = PORT
        self.connection = None
        self.connection_error = False
        self.robot_mode = None
        self.safety_status = None
        self.operational_mode = None
        self.remote_control_status = None

        self.connect()
        self.initialize()

    def connect(self):
        """Create a socket"""
        try:
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.settimeout(5) # Socket will wait 5 seconds till it recieves the response
            self.connection.connect((self.hostname,self.port))

        except socket.error as err:
            print("UR dashboard could not establish connection")
            print(err)
            self.connection_error = True
            

    def disconnect(self):
        """Close the socket"""
        self.connection.close()

    def send_command(self, command, response_delay:float = 0.1):

        # print(">> " + command)

        try:
            if not self.connection:
                self.connect()

            self.connection.sendall((command.encode("ascii") + b"\n")) 
            
            sleep(response_delay) # Wait for response delay
            response = self.connection.recv(4096).decode("utf-8")
                
            if response.find('Connected: Universal Robots Dashboard Server') != -1:
                print("Connected: Universal Robots Dashboard Server")
                response = response[45:]

            print("<< " + response[:-1])

            return response.strip()

        except Exception as err:
            print(err)

    def get_overall_robot_status(self):
        """Get robot status"""
        self.robot_mode = self.get_robot_mode().upper()
        self.operational_mode = self.get_operational_mode().upper()
        self.safety_status = self.get_safety_status()
        self.remote_control_status = self.is_in_remote_control()

    def initialize(self):
        if self.connection_error:
            return
            
        self.get_overall_robot_status()

        if self.safety_status == 'PROTECTIVE_STOP':
            print("Unlocking protective stop")
            self.unlock_protective_stop()

        elif "NORMAL" not in self.safety_status:   #self.safety_status != "ROBOT_EMERGENCY_STOP" or self.safety_status != "SYSTEM_EMERGENCY_STOP":
            print("Restarting safety")
            self.close_safety_popup()
            self.restart_safety()        

        if self.operational_mode == "MANUAL":
            print("Operation mode is currently set to MANUAL, switching to AUTOMATIC")
            self.set_operational_mode("automatic")

        if self.remote_control_status == False:
            print("Robot is not in remote control")
        
        if self.robot_mode == 'RUNNING' and "NORMAL" in self.safety_status:
            print('Robot is initialized')
            return
        
        elif self.robot_mode == "POWER_OFF" or self.robot_mode == "BOOTING" or self.robot_mode == "POWER_ON" or self.robot_mode == "IDLE":
            print("Powering on the robot and releasing brakes")
            self.brake_release()

        return self.initialize()

    def get_robot_mode(self):
        """Return the robot mode"""
        output = self.send_command("robotmode")
        output = output.split(' ')
        try:
            if "\n" in output[1]:
                return output[1].split("\n")[0]
            else:
                return output[1]
        except IndexError:
            print("Depricated output!")
            return output
                
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
        output = self.send_command('brake release')
        sleep(20)
        return output

    def unlock_protective_stop(self):
        return self.send_command('unlock protective stop')

    def close_safety_popup(self):
        return self.send_command('close safety popup')

    def is_in_remote_control(self):
        return self.send_command('is in remote control')

    def restart_safety(self):
        output = self.send_command('restart safety')
        sleep(10)
        self.disconnect()
        self.connect()
        output2 = self.brake_release()
        return output
        
    def get_safety_status(self):
        output = self.send_command('safetystatus')
        output = output.split(' ')
        # print(output)
        try:
            if "\n" in output[1]:
                return output[1].split("\n")[0]
            else:
                return output[1]
        except IndexError:
            print("Depricated output!")
            return output
                
    def get_operational_mode(self):
        return self.send_command('get operational mode')

    def set_operational_mode(self, mode):
        return self.send_command('set operational mode ' + mode)

    def clear_operational_mode(self):
        return self.send_command('clear operational mode')

    def popup(self, message):
        return self.send_command('popup ' + message)

    def close_popup(self):
        return self.send_command('close popup')
    
    def transfer_program(self, local_path:str = None, ur_path:str = "/programs/"):
        if not local_path:
            print("Local file was not provided!")
            return
        try:
            ssh_client = SSHClient()
            ssh_client.load_system_host_keys()
            ssh_client.set_missing_host_key_policy(AutoAddPolicy())
            ssh_client.connect(hostname = self.IP, username = "root", password = "123", disabled_algorithms={'pubkeys': ['rsa-sha2-256', 'rsa-sha2-512']})         
            with SCPClient(ssh_client.get_transport()) as scp:
                scp.put(local_path, ur_path)

        except SSHException as scp_err:
            print("SSH error: " + scp_err)      

        except SCPException as scp_err:
            print("SCP error: " + scp_err)

        else:
            print("UR program "+ local_path + " is transferred to UR onboard " + ur_path)
        
        finally:
            scp.close()
            ssh_client.close()

    def load_program(self, program_path:str):
        return self.send_command("load " + program_path)
    
    def get_program_state(self):
        return self.send_command('programState')
    
    def get_loaded_program(self):
        return self.send_command('get loaded program')
    
    def get_program_run_status(self):
        return self.send_command('running')
    
    def run_program(self):
        return self.send_command('play')
    
    def pause_program(self):
        return self.send_command('pause')
    
    def stop_program(self):
        return self.send_command('stop')


if __name__ == "__main__":
    robot = UR_DASHBOARD("164.54.116.129")
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
    
     
    """
    Bug: Output error message breaks the client state check, causing the client to retry connection 

    [ur5_client-1] [INFO] [1685573134.289546521] [ur5_client.UR5_Client_Node]: {'program_name': 'chemspeed2tecan'}
    [ur5_client-1] [INFO] [1685573134.290071438] [ur5_client.UR5_Client_Node]: None
    [ur5_client-1] [INFO] [1685573134.290490743] [ur5_client.UR5_Client_Node]: chemspeed2tecan
    [ur5_client-1] << NONE
    [ur5_client-1] File not found: /programs/chemspeed2tecan
    [ur5_client-1] << Safetystatus: NORMAL
    [ur5_client-1] << Robotmode: RUNNING
    [ur5_client-1] ['Robotmode:', 'RUNNING']
    [ur5_client-1] << true
    [ur5_client-1] [ERROR] [1685573134.745633959] [ur5_client.UR5_Client_Node]: list index out of range
    [ur5_client-1] [ERROR] [1685573134.746134932] [ur5_client.UR5_Client_Node]: State: ERROR
    [ur5_client-1] [ERROR] [1685573134.746477735] [ur5_client.UR5_Client_Node]: Robot_Mode: RUNNING Safety_Status: RUNNING
    """