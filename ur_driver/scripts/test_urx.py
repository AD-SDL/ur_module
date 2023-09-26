from urx import Robot
from time import sleep
from math import radians, pi, degrees 
# import math3d as m3d

robot = Robot("164.54.116.129")

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
# print(robot.getl())
loc = robot.getj()
print(robot.getl())
tool_dock_l = [-0.30533163571362804, 0.293042569973924, 0.234306520730365, -3.1414391023029085, 0.014564845435757333, 0.0040377171549781125]
tool_dock_j = [2.692000389099121, -2.031496664086813, -1.3187065124511719, -1.3587687772563477, -4.709893051777975, 1.129366159439087]
tool_above_j = [2.691868543624878, -2.01027836422109, -1.1145529747009277, -1.584151407281393, -4.710240427647726, 1.1290664672851562]
tool_above_l = [-0.30521326850056485, 0.29304507597600077, 0.2842680699923231, -3.141487582034991, 0.014625666717814146, 0.0039104466707534005]
# loc[0] += 0.01
# robot.movej([1.4990956783294678, -1.7068420849242152, -1.4129819869995117, -1.5838877163329066, -4.720594708119528, 5.109734535217285], 0.5,0.5)
# robot.movej(tool_above_j,0.5,0.5)
# robot.movel(tool_dock_l,0.2,0.2)
# robot.translate_tool([0,-0.1,0.0],0.2,0.2)
# robot.translate_tool([0,0.1,0.0],0.2,0.2)
# # sleep(1)
# robot.translate_tool([0,0.0,-0.05],0.2,0.2)



# robot.translate_tool([0,0.0,0.05],0.2,0.2)
# tool_dock_l[2]-=0.01
#######################3
# current_orientation =robot.get_orientation()
# euler_angles = current_orientation.to_euler(encoding="xyz")
# print(euler_angles)
# move_rx = (3.14 - abs(euler_angles[0]))

# if euler_angles[1] < 0: 
#     move_ry = abs(euler_angles[1])
# else:
#     move_ry = -(euler_angles[1])


# current_orientation.rotate_xt(move_rx)
# current_orientation.rotate_yt(-move_ry)
# print(move_ry)

# robot.set_orientation(current_orientation, 0.2, 0.2)

##########################################333333
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