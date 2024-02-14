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
#HOST = 'ur5-12idc.xray.aps.anl.gov'
HOST = "164.54.116.129"
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

class PipetError(Exception):
    pass

class PipetteTimeout(Exception):
    pass

class PipetSocketError(Exception):
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
        self._step = 0

    def get_step(self):
        try:
            pos = self._get_var('?0')
            return pos
        except PipetSocketError:
            pos = -1
        timeout = 10
        cnt = 1
        while (pos<0) and (cnt<timeout):
            try:
                pos = self._get_var('?0', trial=cnt)
            except PipetSocketError:
                pos = -1
            time.sleep(0.1*cnt)
            if cnt>1:
                self.disconnect()
#                print(f"Pipet: get_step tried {cnt} times.")
            cnt = cnt+1
            self.connect()
        self._step = pos
        if cnt==timeout:
            raise CommunicationException("Pipet Communication Timeout.")
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

    def get_max_homing_steps(self):
        return self._get_var('?7')

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

    def get_parameter(self, parameter):
        return self._get_var(parameter, isfloat = False)

    def initialize(self):
        self.send_command("z1600A0A10z0", wait=True)
    
    def set_speed(self, start, top, stop):
        self._set_vars({"v":start, "V":top, "c":stop})

    def set_motorcurrent(self, value=50):
        self._set_var("m", value)
    
    def set_step(self, value):
        self.send_command(f"z{value}", wait=True)
    
    def dispense(self, vol=0, percent=0, start=0, speed=0, stop=0):
        cvol = self.get_volume()
        cpos = self._convert_vol_to_step(cvol)
#        print("got the volume.")
        if vol !=0:
            pos = self._convert_vol_to_step(vol)
        else:
            pos = self._convert_percent_to_step(percent)
        par = {"D": pos}
        if cpos-pos<0:
            raise PipetError("Cannot dispense to negative volume.")
        if start !=0:
            par.update({"v":start})
        if speed !=0:
            par.update({"V":speed})
        if stop !=0:
            par.update({"c":stop})
        self._set_vars(par)

        newpos = self.get_step()
        while (newpos > cpos-pos):
            time.sleep(0.1)
            _pos = self.get_step()
            if newpos == _pos:
                break
            else:
                newpos = _pos
        vol = self.get_volume()
        print(f"Pipet is at {vol} uL position.")

    
    def aspirate(self, vol=0, percent=0, start=0, speed=0, stop=0):
        cvol = self.get_volume()
        cpos = self._convert_vol_to_step(cvol)
        if vol !=0:
            pos = self._convert_vol_to_step(vol)
        else:
            pos = self._convert_percent_to_step(percent)
        if cpos+pos > self._max_position:
            raise PipetError("Cannot aspirate more than the max.")
        par = {"P": pos}
        if start !=0:
            par.update({"v":start})
        if speed !=0:
            par.update({"V":speed})
        if stop !=0:
            par.update({"c":stop})
        self._set_vars(par)

        newpos = self.get_step()
        while (newpos < cpos+pos):
            time.sleep(0.1)
            _pos = self.get_step()
            if newpos == _pos:
                break
            else:
                newpos = _pos

        # while (self.get_step() < cpos+pos):
        #     time.sleep(0.1)

        vol = self.get_volume()
        print(f"Pipet is at {vol} uL position.")

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

    def connect(self, hostname: str=HOST, port: int = 54321, socket_timeout: float = 1) -> None:
        """Connects to a pipet at the given address.
        """
        if hasattr(self, hostname):
            con = (self.hostname, self.port)
        else:
            con = (hostname, port)
        self.address = con
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(con)
        self.socket.settimeout(socket_timeout)

    def reconnect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(self.address)
        self.socket.settimeout(1)
        #self.socket.connect(self.address)

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
        status, val, data = self.query(cmd)
        errcheck, notbusy = self._status_check(status)
#         with self.command_lock:
#             self.socket.sendall(cmd.encode(self.ENCODING))
#             time.sleep(0.1)
#             ans = ""
#             data = ""
#             try:
#                 data = self.socket.recv(1024)
# #                print(data, "Normal")
#             except CommunicationException:
#                 data = ""
#             except ConnectionAbortedError:
#                 self.connect()
#                 data = ""
#             except socket.timeout:
#                 print("socket timeout in set_var")
#                 time.sleep(0.1)
#                 data = ""
#             try:
#                 ans = self._get_answer(data)
#             except:
# #                print(data, "Decode error on line 347")
#                 pass
#             if len(ans)>0:
#                 errcheck, notbusy = self._status_check(ans)
#             else:
#                 wait = False
#                 errcheck = False

        t = time.time()
        timeout = 5
        if wait:
            while not notbusy:
                time.sleep(0.1)
                errcheck, notbusy = self.get_status()
                if time.time()-t > timeout:
#                    print("get_status timeout in set_vars.")
                    break
        return errcheck
            # if noerror:
            #     while notbusy:
    
    def get_status(self):
        cmd = "/1Q\r"
        readdone = False
        timeout = 10
        cnt = 1
        while ((not readdone) and (cnt < timeout)):
            try:
                status, _, _ = self.query(cmd)
                errcheck, busycheck = self._status_check(status)
                readdone = True
            except CommunicationException:
                time.sleep(0.1*cnt)
            except socket.timeout:
                time.sleep(0.1*cnt)
            except ConnectionAbortedError:
                self.connect()
                time.sleep(0.1*cnt)
            cnt = cnt + 1
        if cnt==timeout:
            raise CommunicationException("Time out.")
        return (errcheck, busycheck)
    
    def query(self, cmd, trial = 10, timeofsleep = 0.1):
        readdone = False
        cnt = 1
        data = ""
        status = ""
        with self.command_lock:
            while (readdone==False) and (cnt<trial):
                try:
                    self.socket.sendall(cmd.encode(self.ENCODING))
                    time.sleep(timeofsleep)
                    data = self._recv()
                    readdone = True
                except CommunicationException:
                    time.sleep(0.1*cnt)
                except ConnectionAbortedError:
                    self.reconnect()
                except PipetteTimeout as err:
                    print(err)
                    return "", "", data
                except BrokenPipeError as err2:
                    print(err2)
                    self.reconnect()
                cnt = cnt+1
            if cnt==trial:
                raise PipetteTimeout
        if len(data)>0:
            status, val = self._get_answer(data)
        else:
            val = ""
        return status, val, data

    def _recv(self):
        data = b''
        k = b'/'
        timeout = 5
        t = time.time()
        while True:
            try:
                k = self.socket.recv(1)
                if len(k)>0:
                    if isinstance(k, bytes):
                        data += k
                    if k[0] == 10:
                        break
                else:
                    break
                if (time.time()-t>timeout):
                    raise PipetteTimeout
            except socket.timeout:
                raise PipetteTimeout
        return data

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

    def _get_var(self, variable: str, trial=3, isfloat=True):
        """Retrieve the value of a variable from the pipet, blocking until the
        response is received or the socket times out.
        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer.
        """
        # atomic commands send/rcv
        cmd = "/1"
        cmd += f"{variable}"
        cmd += '\r'
        #readdone = False
        #cnt = 1
        #data = ""
        status, val, data = self.query(cmd)
        resp, notbusy = self._status_check(status)
        if resp == False:
            raise PipetSocketError
        if isfloat == False:
            return val
        try:
            return float(val)
        except ValueError:
#            print(data, "Pipet: Return decode error. ans should be a number string.")
            raise PipetSocketError

    def _get_answer(self,data):
        if len(data)==0:
            raise CommunicationException
        dt = data.split(b'\n')
        if len(dt)==1:
            raise CommunicationException
        # return should start from /x (id of the controller) and ends with \n
        data = dt[len(dt)-2]
        data, _ = data.split(b'\x03')
        if data[0] != 47:
            raise CommunicationException
        status = data[2]
        status = chr(status)
        if len(data)>2:
            value = data[3:]
            value = value.decode(self.ENCODING)
        else:
            value = None
        return status, value

    def _decode_answer(self, data):
        if len(data)==0:
            raise CommunicationException
        dt = data.split(b'\n')
        if len(dt)==1:
            raise CommunicationException
        # return should start from /x (id of the controller) and ends with \n
        data = dt[len(dt)-2][1:]
        data, _ = data.split(b'\x03')
        data = data.decode(self.ENCODING)
        ans = data[2:]
        return ans
    
    def _status_check(self, data):
        resp = ""
        notbusy = False
        if len(data) == 0:
            return (False, notbusy)
#            raise CommunicationException
        if data == "`":
#            print('Not busy')
            return ("", True)
        if data == "@":
#            print('Busy')
            return ("", False)
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
        raise PipetError(resp)
#        try:
#            print(resp, notbusy)
#        except:
#            print(data, "status_check")
#        return (False, notbusy)
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
    # a.connect(hostname="164.54.116.129")
    a.connect(hostname="192.168.1.102")
    # time.sleep(5)
    # a.initialize()
    # time.sleep(5)
    # print(a.get_speed_start(), a.get_speed_stop(),a.get_speed())
    a.dispense(vol=25)
    # time.sleep(5)
    # print(a.get_step())
    # a.dispense(vol=2)
    # cmd = '/1'
    # if len(sys.argv)<2:
    #     cmd += 'z1600A0A10z0R'
    # else:
    #     cmd += sys.argv[1]
    # print(a.send_command("z1600A0A10z0R"))
    a.disconnect()