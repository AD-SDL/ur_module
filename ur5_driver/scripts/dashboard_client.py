import socket
import time

class DashboardClient:
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.settimeout(2)
        self.s.connect(('192.168.1.102', 29999))

    def send_command(self, cmd):
        cmd = cmd + '\n'
        print(cmd)
        self.s.sendall(cmd.encode())
        time.sleep(5)
        rcvd = self.s.recv(4096)
        print(rcvd)
        return rcvd
    
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


# while (1):
#  sendCommand('programState')

if __name__ == "__main__":
    robot = DashboardClient()
    robot.power_on()
    robot.brake_release()