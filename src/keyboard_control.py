from ur_driver.ur import UR
import time
from pynput.keyboard import Key, Listener, Controller 
from pynput import keyboard
import threading
import numpy
import math
import argparse
parser = argparse.ArgumentParser(
                    prog='UR keyboard',
                    description='control ur arm using keyboard',
                    )
parser.add_argument('-u', '--url', default="146.137.240.38")
parser.add_argument('-l', '--lin_speed', default=0.01)
parser.add_argument('-r', '--rad_speed', default=0.01)
args = parser.parse_args()
robot = UR(args.url)
move_speed = float(args.lin_speed)
move_speed_radial = float(args.rad_speed)
print(robot.ur_connection.getl())
robot.velocity = 0.8
robot.acceleration = 0.8
flag = False


def move_robot(direction, sign):
    global flag
    flag = True
    pos = [0, 0, 0]
    pos[direction] = sign * move_speed
    robot.ur_connection.translate_tool(pos, 1, 1)
    flag = False
def move_robot_j(direction, sign):
    global flag
    flag = True
    pos = robot.ur_connection.getj()
    print(pos)
    pos[direction] += sign * move_speed_radial
    robot.ur_connection.movej(pos, 1, 1)
    flag = False
exit = False
def on_press(key):
    global flag
    # print(robot.ur_connection.get_orientation())
    # return
    print("key detected: " + str(key))
    if not flag:
      if key == Key.up:
        move_robot(1, 1)
      elif key == Key.down:
        move_robot(1, -1)
      elif key == Key.right:
        move_robot(0, -1)
      elif key == Key.left:
        move_robot(0, 1)
      elif key.char == "w":
        move_robot(2, 1)
      elif key.char == "s":
        move_robot(2, -1)
      elif key.char == "8":
          move_robot_j(5, 1)
      elif key.char == "2":
          move_robot_j(5, -1)     
      elif key.char == "4":
         move_robot_j(4, 1)
      elif key.char == "6":
         move_robot_j(4, -1)
      # elif key.char == "7":
      #   move_robot(5, 1)
      # elif key.char == "9":
      #   move_robot(5, -1) 
      elif key.char == "l":
        # pos = robot.ur_connection.getj()
        # pos[5] = 0
        # robot.ur_connection.movej(pos, 0.8, 0.8)
        pos = robot.ur_connection.getl()  
      
        pos[3] = math.pi/2
        pos[4] = 0
        pos[5] = 0

        robot.ur_connection.movel(pos, acc=1, vel=1)
      elif key.char == "f":
        # pos = robot.ur_connection.getj()
        # pos[5] = 0
        # robot.ur_connection.movej(pos, 0.8, 0.8)
        pos = robot.ur_connection.set_freedrive(True, 600)  
listener = keyboard.Listener(on_press=on_press)
listener.start()
# while not exit:
#   pos = robot.ur_connection.getl()
#   print(dirs)
#   pos = numpy.array(pos) + numpy.array(dirs)*0.01
#   robot.ur_connection.movel(pos.tolist())