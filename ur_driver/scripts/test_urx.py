from urx import Robot
from time import sleep
from math import radians, pi, degrees 
import math3d as m3d

robot = Robot("192.168.1.100")

# Set TCP
# robot.set_tcp((0, 0, 0, 0, 0, 0))

# # Set Payload
# robot.set_payload(weight=0.9)

# # Set robot to freedrive for 60 seconds
# robot.set_freedrive()

# Get robot current location based on joint angles
print(robot.getj()) #radians
degree_angles = [degrees(i) for i in robot.getj()] 
print(degree_angles) #degrees

# Get robot current location based on cartesian coordinates (X Y Z, RX RY RZ)
print(robot.getl())

# Joint movement using joint angles in radians
# robot.movej([radians(-97.88), radians(-119.54), radians(-41.06), radians(-109.47), radians(90.48), radians(82.48)], acc = 0.1, vel = 0.1)

# # Linear movement, using cartesian coordinates with respect to base reference frame
# robot.movel([-0.22575, -0.65792, 0.39771, 2.216, 2.196, -0.043], acc=0.01, vel=0.01)

# # Move tool and keep orientation with respect to base reference frame
# robot.translate([0.1, 0, 0], acc = 0.1, vel = 0.1)  

# # Move tool and keep orientation with respect to tool reference frame
# robot.translate_tool([0.1, 0, 0], acc = 0.1, vel = 0.1)  

# # Create a matrix for our tool tcp
# mytcp = m3d.Transform()  
# mytcp.pos.z = 0.18
# mytcp.orient.rotate_zb(pi/3)
# robot.set_tcp(mytcp)
# sleep(0.2)

# # Get current pose, transform it and move robot to new pose
# trans = robot.get_pose()  # get current transformation matrix (tool to base)
# trans.pos.z += 0.3
# trans.orient.rotate_yb(pi/2)
# robot.set_pose(trans, acc=0.5, vel=0.2)  # apply the new pose

# #Or only work with orientation part
# o = robot.get_orientation()
# o.rotate_yb(pi)
# robot.set_orientation(o)

robot.close()