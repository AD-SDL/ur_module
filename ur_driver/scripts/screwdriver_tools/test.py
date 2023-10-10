import socket
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

ROBOT_IP = "164.54.116.129"

# Initialize RTDE control and receive interfaces
control = RTDEControlInterface(ROBOT_IP)
receive = RTDEReceiveInterface(ROBOT_IP)

# You can use the control and receive interfaces for RTDE operations
# For example, get the robot state
# actual_q = receive.getActualQ()
# print("Actual joint positions:", actual_q)

# To send a URScript command, you can use the standard socket mechanism
urscript_command = """
rq_screw_activate()\n
rq_screw_turn(1,1,3600,250,True,9)\n
sleep(5)\n
end
"""
command = "textmsg('Hello from URScript!')\n"


# Create a socket connection and send the command
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((ROBOT_IP, 30003))
    s.sendall(command.encode())
    f = open("/home/rpl/wei_ws/src/ur_module/ur_driver/scripts/screwdriver_tools/screwdriver.script", "rb")
    l = f.read(1024)

    while (l):
        s.send(l)
        l = f.read(1024)

# Always remember to disconnect the RTDE interfaces when done
control.disconnect()
receive.disconnect()
