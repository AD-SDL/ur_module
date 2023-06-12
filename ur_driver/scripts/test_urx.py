from urx import Robot
import time
from math import radians

robot = Robot("192.168.1.102")

time.sleep(0.2)

# goal position: (-0.22575, -0.65792, 0.39771, 2.216, 2.196, -0.043)

#robot.movej((radians(-97.88), radians(-119.54), radians(-41.06), radians(-109.47), radians(90.48), radians(82.48)), 0.01, 0.01)

#robot.movel((0, 0, -0.05, 0, 0, 0), 0.05, 0.05, relative=True) 

# # move above goal position
robot.movel([-0.22575, -0.65792, 0.39771 + 0.20, 2.216, 2.196, -0.043], 0.01, 0.01)

# move to goal position
#robot.movel((-0.22575, -0.65792, 0.37771, 2.216, 2.196, -0.043), 0.05, 0.05)

# # move back above goal position
# robot.movel((-0.22575, -0.65792, 0.39771 + 0.20, 2.216, 2.196, -0.043), 0.01, 0.01)