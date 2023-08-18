'''
This is a python code to control the Z-series pump of Tricontinent.
It assumes two things:
1. the pump is connected to the Tool I/O of a UR robot, which requires wiring of 4 pins of the Z-pump control board.
Example, when a Lumberg RKMV 8-354 is used, its gray, red, brown, and white cables should go 24V, GND, RS485B, and RS485A pins.
According to a UR3 manual, gray for Power, red for GND, brown for Analog input 3, and white for Analog input 2.
2. a URcap for RS485 daemon is required. Download and install from https://forum.universal-robots.com/t/establish-rs485-connection-with-tool/14821/5

Author: Byeongdu Lee (Argonne National Laboratory)
Date: July, 2023
'''
import socket
import threading
import time
from typing import Union, OrderedDict
import sys
HOST = 'ur5-12idc.xray.aps.anl.gov'
'''
# set commands
Initialization Commands
V (5-6000)  Set top speed in half steps per second. Default = 1400
v (0-1000)  Set start speed in half steps per second. Default = 0
c (50-2700) Set stop speed in half steps per second. Default = 900
S (0-40)    Set top speed using speed codes. Default = 11
L (1-20)    Set acceleration factor (accel="L"*2.5 kHz/sec.) Default = 14
m (0-100)   Sets the Motor Run current in a % of maximum (500mA peak). For example, m50R will set the run
current to 50% of its maximum (250mA). Similar to the [u2] command, only this setting will be lost when
the power is cycled, or it is volatile. Whereas the [u2] is non-volatile.

h (0-100) Sets the Motor Hold current in a % of maximum (500mA peak). For example, h10R will set the hold
current to 10% of its maximum (50mA). Similar to the [u1] command, only this setting will be lost when
the power is cycled, or it is volatile. Whereas the [u1] is non-volatile.

N (0-1) N = 0, all motor positions are in half steps; N=1, positions are in micro- steps, 8 micro-steps per half-step.
Default N = 0 or half-step mode.

K (0-31) Sets number of backlash steps. Default K = 0.
k (0-80) Syringe dead volume. After initialization, the plunger will move this many half-steps to minimize the dead
volume. Default k = 0.

u (n_X) Will load pump configuration and calibration info into the internal EEPROM. Note, these parameters are
only read on power up. Thus they will only take effect when the power is cycled. Note this command,
unlike the Set commands, does not require an [R] to execute.
1. (1_XXX) Motor holding current, 0 –100% (100% = 500mA peak)
2. (2_XXX) Motor running current, 0 –100%
3. (3_XXX) Max home steps in 100 half-step increments (1-250)
4. (4_XXX) Max home speed in 100 half-steps/sec increments (1-100)
5. (5_XXX) Homing back-steps in 100 half-steps increments(1-250)
6. (6_XXX) Default max V in 100 half-steps/sec increments (1-100)
7. (7_XXX) Max plunger stroke in 100 half-step increments (1–250)
8. (8_XXX) Home position at top, X= 1, at bottom X= 0
9. (9_XXX) Number of user settable outputs(0–4)
10. (10_X) No homing opto, X=0. Homing opto installed, X=1
11. (11_X) Stall guard level for no-opto homing (1-7)
12. (12_X) Solenoid daughter installed, X=1, not installed X=0
13. (13_X) CAN Bus option installed, X=1, not installed X=0
14. (14_X) Number of backlash steps
15. (15_X) Motor winding for LT, X=1, for Z-Pump X=0
16. (16_X) Home sensor polarity low = blocked, X=1, low = unblocked X=0
17. (17_X) Self test mode string

z (0-1600) Sets current position and initializes the plunger to the value defined by the operand.
(0-12,800 in micro-step mode)


Movement Commands
Note: The limits below assume 1600 half-steps per stroke. The limits for micro-step mode [N =1] will always be 8X that of
the half-step [N=0] mode.
Report Commands
A (0-1600) Move motor to absolute position (0-12,800 in micro-step mode).
a (0-1600) Same as [A], but will give a non-busy status code.
P (0-1600) Move motor relative number of steps in the aspirate direction (0-12,800 in micro-step mode).
p (0-1600) Same as [P], but will give a non-busy status code.
D (0-1600) Move motor relative number of steps in the dispense direction (0-12,800 in micro-step mode).
d (0-1600) Same as [D], but will give a non-busy status code.
Q Returns the status character. (refer to DT Protocol, Answer Block)
? or ?0 Returns current absolute position.
?1 Returns start speed.
?2 Returns top speed.
?3 Returns stop speed.
?7 Reports max homing steps.
?8 Reports homing speed.
?9 Reports homing back steps.
?10 Reports syringe dead volume.
?11 Reports backlash steps.
?24 Reports syringe dead volume.
?25 Reports motor hold current.
?26 Reports motor run current.
? (30-44) Reports user program strings loaded into external or user EEPROM.
?30 reports string 0, ?31 string 1, and so on.
& Returns the firmware revision and date.
F Reports command buffer status. If the buffer is empty, the pump returns status code
'''


## Exception handling....
class CommunicationException(Exception):
    pass


class PipetteDriver():
    # host = 'robot_ip'
    # port = 54321# Remains the same, because it is specified as default port in URCaps code
    # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # s.connect((host, port))
    # s.sendall(b'Hello rs485 port from ash')
    # data = s.recv(1024)
    # s.close()
    # print('Received', repr(data))

    ENCODING = 'UTF-8'  # ASCII and UTF-8 both seem to work
    NoError_NotBusy = "`"
    NoError_Busy = "@"
        # InitializationError_NotBusy = "a"
        # InitializationError_Busy = "A"
        # InvalidCommand_NotBusy = "b"
        # InvalidCommand_Busy = "B"
        # InvalidOperand_NotBusy = "c"
        # InvalidOperand_Busy = "C"
        # DeviceNotInitialized_NotBusy = "g"
        # DeviceNotInitialized_Busy = "G"
        # CommandOverflow_NotBusy = "o"
        # CommandOverflow_Busy = "O"

    def __init__(self):
        """Constructor."""
        self.socket = None
        self.command_lock = threading.Lock()
        self._min_position = 0
        self._max_position = 1600
        self._min_start_speed = 0
        self._max_start_speed = 1000
        self._min_top_speed = 5
        self._max_top_speed = 6000
        self._min_stop_speed = 50
        self._max_stop_speed = 2700
        self._min_backlash_step = 0
        self._max_backlash_step = 31
        self._min_deadvolume = 0
        self._max_deavvolume = 80
        self.volume = 200 # 200uL in full volume.
        # for this volume resolution is 0.1905 uL
        self._comm_setting = {"baud_rate": 9600, 
                      "parity": 0,
                      "stop_bits": 1,
                      "rx_idle_chars": 1.5,
                      "tx_idle_chars": 3.5}

    def get_step(self):
        pos = self._get_var('?0')
        return pos
    
    def get_step_percent(self):
        pos = self.get_step()
        return self._convert_step_to_percent(pos)
    
    def get_volume(self):
        pos = self.get_step()
        return self._convert_step_to_volume(pos)
    
    def get_speed_start(self):
        return self._get_var('?1')

    def get_speed_start_percent(self):
        return self.get_speed_start()/self._max_start_speed*100

    def get_speed(self):
        return self._get_var('?2')

    def get_speed_percent(self):
        return self.get_speed()/self._max_top_speed*100

    def get_speed_stop(self):
        return self._get_var('?3')

    def get_speed_stop_percent(self):
        return self.get_speed_stop()/self._max_stop_speed*100

    def get_dead_volume(self):
        return self._get_var('?10')

    def get_backlash(self):
        return self._get_var('?11')

    def get_dead_volume2(self):
        return self._get_var('?24')

    def get_hold_current(self):
        return self._get_var('?25')

    def get_run_current(self):
        return self._get_var('?26')

    def initialize(self):
        self.send_command("z1600A0A10z0", wait=True)
    
    def set_speed(self, start, top, stop):
        self._set_vars({"v":start, "V":top, "c":stop})

    def set_motorcurrent(self, value=50):
        self._set_var("m", value)
    
    def dispense(self, vol=0, percent=0, start=0, speed=0, stop=0):
        if vol !=0:
            pos = self._convert_vol_to_step(vol)
        else:
            pos = self._convert_percent_to_step(percent)
        par = {"D": pos}
        if start !=0:
            par.update({"v":start})
        if speed !=0:
            par.update({"V":speed})
        if stop !=0:
            par.update({"c":stop})
        self._set_vars(par)
    
    def aspirate(self, vol=0, percent=0, start=0, speed=0, stop=0):
        if vol !=0:
            pos = self._convert_vol_to_step(vol)
        else:
            pos = self._convert_percent_to_step(percent)
        par = {"P": pos}
        if start !=0:
            par.update({"v":start})
        if speed !=0:
            par.update({"V":speed})
        if stop !=0:
            par.update({"c":stop})
        self._set_vars(par)

    def _convert_vol_to_step(self, vol): #mL
        percent = vol/self.volume*100
        return self._convert_percent_to_step(percent)

    def _convert_step_to_volume(self, value):
        vol = value/self._max_position*self.volume
        return vol

    def _convert_step_to_percent(self, value):
        percent = value/self._max_position*100
        return percent
    
    def _convert_percent_to_step(self, percent):
        value = percent/100*self._max_position
        return int(value)
    
    def stop(self):
        self.send_command("T")

    def move(self, newpos, relative=True):
        if relative:
            pos = self.get_step()
            newpos = pos + newpos
        return self._set_var('A', newpos)

    def connect(self, hostname: str=HOST, port: int = 54321, socket_timeout: float = 2.0) -> None:
        """Connects to a pipette at the given address.
        """
        if hasattr(self, hostname):
            con = (self.hostname, self.port)
        else:
            con = (hostname, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(con)
        self.socket.settimeout(socket_timeout)


    def disconnect(self) -> None:
        """Closes the connection with the gripper."""
        try:
            self.socket.close()
        except:
            print("Connection has never been made.")
    
    def var_test(self, var_dict: OrderedDict[str, Union[int, float]], wait=True):
        for variable, value in var_dict.items():
            print(f"{variable}{str(value)}.")

    def _set_vars(self, var_dict: OrderedDict[str, Union[str, int, float]], timeout = 10, wait=True):
        """set variables or parameters.
        :param var_dict: Dictionary of variables to set (variable_name, value).
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        # construct unique command
        cmd = "/1"
        for variable, value in var_dict.items():
            if type(value) == str:
                val = value
            else:
                val = str(value)
            cmd += f"{variable}{val}"
        cmd += 'R\r'  # R for execution and new line is required for the command to finish
        # atomic commands send/rcv
        #timeout = 10
        readdone = False
        cnt = 1
        timeout = 10
        with self.command_lock:
            while ((not readdone) and (cnt < timeout)):
                self.socket.sendall(cmd.encode(self.ENCODING))
                time.sleep(0.05)
                try:
                    data = self.socket.recv(1024)
                    readdone = True
                except:
                    self.connect()
                    cnt = cnt + 1
                    # self.socket.sendall(cmd.encode(self.ENCODING))
                    # time.sleep(0.1)
                    # data = self.socket.recv(1024)
                    print(f"pipette status checking has been iterated {cnt} times without success.")
                ans = self._decode_answer(data)
                if len(ans) == 0:
                    readdone = False
            t = time.time()
            errcheck, notbusy = self._status_check(ans)
        if wait:
            while not notbusy:
                time.sleep(0.1)
                errcheck, notbusy = self.get_status()
                if time.time()-t > timeout:
                    print("Timeout")
                    break
        return errcheck
            # if noerror:
            #     while notbusy:
    
    def get_status(self):
        cmd = "/1Q\r"
        readdone = False
        timeout = 10
        cnt = 1
        with self.command_lock:
            while ((not readdone) and (cnt < timeout)):
                self.socket.sendall(cmd.encode(self.ENCODING))
                time.sleep(0.05)
                try:
                    data = self.socket.recv(1024)
                    ans = self._decode_answer(data)
                    errcheck, busycheck = self._status_check(ans)
                    readdone = True
                except:
                    self.connect()
                    cnt = cnt + 1
            if cnt==timeout:
                raise CommunicationException
        return (errcheck, busycheck)
    
    def send_command(self, command, value="", timeout=10, wait=True):
        return self._set_var(command, value=value, timeout=timeout, wait=wait)
    
    def _set_var(self, variable: str, value: Union[str, int, float], timeout = 10, wait=True):
        """set a single variable.
        :param variable: Variable to set.
        :param value: Value to set for the variable.
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        return self._set_vars(OrderedDict([(variable, value)]), timeout = timeout, wait=wait)

    def _get_var(self, variable: str):
        """Retrieve the value of a variable from the pipette, blocking until the
        response is received or the socket times out.
        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer.
        """
        # atomic commands send/rcv
        cmd = "/1"
        readdone = False
        timeout = 5
        cnt = 1
        with self.command_lock:
            cmd += f"{variable}"
            cmd += '\r'
            while ((not readdone) and (cnt<timeout)):
                self.socket.sendall(cmd.encode(self.ENCODING))
                time.sleep(0.05)
                try:
                    data = self.socket.recv(1024)
                    readdone = True
                except:
                    self.connect()
                    cnt = cnt + 1
        if len(data)>0:
            ans = self._decode_answer(data[1:])
            return float(ans)
        else:
            raise CommunicationException

    def _decode_answer(self, data):
        if len(data)==0:
            raise CommunicationException
        data, _ = data.split(b'\x03')
        data = data.decode(self.ENCODING)
        ans = data[2:]
        return ans
    
    def _status_check(self, data):
        if len(data) == 0:
            raise CommunicationException
        if data == "`":
#            print('Not busy')
            return (True, True)
        if data == "@":
#            print('Busy')
            return (True, False)
        if data.isupper():
            #isbusy = "Busy"
            notbusy = True
        else:
            #isbusy = "NotBusy"
            notbusy = False
        data = data.lower()
        if data == 'a':
            resp = "Initialization Error"
        if data == 'b':
            resp = "Invalid Command"
        if data == 'c':
            resp = "Invalid Operand"
        if data == 'g':
            resp = "Device Not Initialized"
        if data == 'o':
            resp = "Command Overflow"
        if data == 'i':
            resp = "Plunger Overload"
        if data == 'h':
            resp = "CAN Bus failure"
        print(resp, notbusy)
        return (False, notbusy)
        # NoError_NotBusy = "`"
        # NoError_Busy = "@"
        # InitializationError_NotBusy = "a"
        # InitializationError_Busy = "A"
        # InvalidCommand_NotBusy = "b"
        # InvalidCommand_Busy = "B"
        # InvalidOperand_NotBusy = "c"
        # InvalidOperand_Busy = "C"
        # DeviceNotInitialized_NotBusy = "g"
        # DeviceNotInitialized_Busy = "G"
        # CommandOverflow_NotBusy = "o"
        # CommandOverflow_Busy = "O"
            
    @staticmethod
    def _is_ack(data: str):
        return data == b'ack'    

if __name__ == "__main__":
    a = PipetteDriver()
    a.connect(hostname="164.54.116.129")
    a.initialize()
    # print(a.get_speed_start(), a.get_speed_stop(),a.get_speed())
    # a.aspirate(vol=20)
    # a.dispense(vol=20)
    # cmd = '/1'
    # if len(sys.argv)<2:
    #     cmd += 'z1600A0A10z0R'
    # else:
    #     cmd += sys.argv[1]
    # print(a.send_command("z1600A0A10z0R"))
    a.disconnect()