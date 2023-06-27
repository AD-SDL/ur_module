import socket
import time
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(2)
s.connect(('192.168.50.82', 29999))
def sendCommand(cmd):
 cmd = cmd + '\n'
 print(cmd)
 s.sendall(cmd.encode())
#  time.sleep(5)
 rcvd = s.recv(4096)
 print(rcvd)
while (1):
 sendCommand('power on')