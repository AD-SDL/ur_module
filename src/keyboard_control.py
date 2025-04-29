"""
UR Robot Keyboard Step Control

Mimics teach pendant behavior: each key press moves the robot a small step using `movel()` for translation/orientation,
and now supports rotating joints one by one with dedicated keys.
Includes basic error recovery for 'robot stopped' errors.
Now tuned for maximum speed and responsiveness.
"""

import argparse
import math
import signal
import time

from pynput import keyboard
from pynput.keyboard import Key
from ur_interface.ur import UR
from urx.urrobot import RobotException

parser = argparse.ArgumentParser(description="Teach Pendant Style Keyboard Control for UR Robot")
parser.add_argument("-u", "--url", default="146.137.240.38", help="UR robot IP address")
parser.add_argument("-s", "--step", default=0.01, type=float, help="Step size in meters or radians")
parser.add_argument("-j", "--joint_step", default=0.05, type=float, help="Step size for joint rotations in radians")
parser.add_argument("--speed", default=2.0, type=float, help="Speed for movements (m/s or rad/s)")
args = parser.parse_args()

robot = UR(args.url)
step = args.step
joint_step = args.joint_step
speed = args.speed

exit_program = False


# Unified movement function (position and orientation)
def step_move(dx=0, dy=0, dz=0, rx=0, ry=0, rz=0):
    """
    Move the robot in a specified direction with a given speed.
    """
    try:
        pos = robot.ur_connection.getl()
        pos[0] += dx
        pos[1] += dy
        pos[2] += dz
        pos[3] += rx
        pos[4] += ry
        pos[5] += rz
        robot.ur_connection.movel(pos, acc=speed, vel=speed)
    except RobotException as e:
        print("[RobotException]", e)
        print("Attempting to resume from protective stop...")
        robot.ur_dashboard.unlock_protective_stop()
        time.sleep(1)
    except Exception as e:
        print("[ERROR]", e)


# Joint movement function
def step_joint(joint_idx=0, sign=1):
    """
    Rotate a specific joint by a small step.
    """
    try:
        joints = robot.ur_connection.getj()
        joints[joint_idx] += sign * joint_step
        robot.ur_connection.movej(joints, acc=speed, vel=speed)
    except RobotException as e:
        print("[RobotException]", e)
        print("Attempting to resume from protective stop...")
        robot.ur_dashboard.unlock_protective_stop()
        time.sleep(1)
    except Exception as e:
        print("[ERROR]", e)


# Function to set gripper facing straight down
def set_gripper_down():
    """
    Set the gripper to face straight down (perpendicular to the ground).
    """
    try:
        pos = robot.ur_connection.getl()
        pos[3] = math.pi
        pos[4] = 0
        pos[5] = 0
        robot.ur_connection.movel(pos, acc=speed, vel=speed)
        print("Gripper now facing down (perpendicular to ground).")
    except Exception as e:
        print("[ERROR]", e)


# Display current pose and joint angles
def print_current_location():
    """
    Print the current TCP pose and joint angles.
    """
    try:
        tcp = robot.ur_connection.getl()
        joints = robot.ur_connection.getj()
        print("\nCurrent TCP Pose:")
        print([round(p, 6) for p in tcp])
        print("Current Joint Angles:")
        print([round(j, 6) for j in joints])
    except Exception as e:
        print("[ERROR] Failed to get robot state:", e)


# Key press handling
def on_press(key):
    """
    Handle key press events to control the robot.
    """
    try:
        if key == Key.up:
            step_move(dy=step)
        elif key == Key.down:
            step_move(dy=-step)
        elif key == Key.left:
            step_move(dx=step)
        elif key == Key.right:
            step_move(dx=-step)
        elif key.char == "w":
            step_move(dz=step)
        elif key.char == "s":
            step_move(dz=-step)
        elif key.char == "j":
            step_move(rz=step)
        elif key.char == "l":
            step_move(rz=-step)
        elif key.char == "u":
            step_move(rx=step)
        elif key.char == "o":
            step_move(rx=-step)
        elif key.char == "m":
            step_move(ry=step)
        elif key.char == ",":
            step_move(ry=-step)

        # Joint control
        elif key.char == "1":
            step_joint(0, 1)
        elif key.char == "2":
            step_joint(0, -1)
        elif key.char == "3":
            step_joint(1, 1)
        elif key.char == "4":
            step_joint(1, -1)
        elif key.char == "5":
            step_joint(2, 1)
        elif key.char == "6":
            step_joint(2, -1)
        elif key.char == "7":
            step_joint(3, 1)
        elif key.char == "8":
            step_joint(3, -1)
        elif key.char == "9":
            step_joint(4, 1)
        elif key.char == "0":
            step_joint(4, -1)
        elif key.char == "-":
            step_joint(5, 1)
        elif key.char == "=":
            step_joint(5, -1)

        elif key.char == "p":
            print_current_location()
        elif key.char == "g":
            set_gripper_down()
        elif key.char == "f":
            robot.ur_connection.set_freedrive(True, 600)
            print("Freedrive mode ON")
        elif key == Key.esc:
            global exit_program
            exit_program = True
            return False
    except AttributeError:
        pass
    except Exception as err:
        print("[ERROR]", err)


# Cleanup on Ctrl+C
def graceful_exit(sig, frame):
    """
    Handle graceful exit on Ctrl+C.
    """
    print("\nStopping robot and exiting...")
    robot.ur_connection.stopl(2)
    exit(0)


signal.signal(signal.SIGINT, graceful_exit)

print("Keyboard control started. Press ESC to quit.")
print("Arrow keys = X/Y movement.")
print("W/S = Z-axis.")
print("U/O = Roll (Rx), J/L = Yaw (Rz), M/, = Pitch (Ry).")
print("Keys 1-0,-,= to rotate individual joints.")
print("'p' to print current pose, 'g' to face gripper down, 'f' for freedrive.")

listener = keyboard.Listener(on_press=on_press)
listener.start()
listener.join()
