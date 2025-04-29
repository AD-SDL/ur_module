"""
UR Robot Keyboard Control Script

Control a UR robot arm using the keyboard. Supports linear (movel) movement by default,
and toggles to joint (movej) control using a keyboard key.
"""

# Standard Library Imports
import argparse
import math
import threading
import time

# Third-Party Imports
from pynput import keyboard
from pynput.keyboard import Key

# Local Imports
from ur_interface.ur import UR

# Argument Parsing
parser = argparse.ArgumentParser(
    prog='UR Keyboard Control',
    description='Control UR robot arm using keyboard inputs.'
)
parser.add_argument('-u', '--url', default="146.137.240.38", help='UR robot IP address')
parser.add_argument('-l', '--lin_speed', default=0.01, type=float, help='Linear speed in m/s')
parser.add_argument('-r', '--rad_speed', default=0.01, type=float, help='Radial speed in rad/s')
args = parser.parse_args()

# Robot Initialization
robot = UR(args.url)
move_speed = args.lin_speed
move_speed_radial = args.rad_speed

# Global Flags
velocity = [0, 0, 0, 0, 0, 0]
freedrive_mode = False
joint_mode = False
exit_program = False

# Robot Parameters
robot.acceleration = 0.8

# Function Definitions
def move_robot(direction: int, sign: int) -> None:
    """Move the robot linearly along a specified axis."""
    velocity[direction] = sign * move_speed

def move_robot_joint(direction: int, sign: int) -> None:
    """Move the robot joint by adjusting joint angles."""
    pos = robot.ur_connection.getj()
    pos[direction] += sign * move_speed_radial
    robot.ur_connection.movej(pos, acc=1.0, vel=1.0)

def toggle_freedrive() -> None:
    """Toggle freedrive mode on or off."""
    global freedrive_mode
    freedrive_mode = not freedrive_mode
    if freedrive_mode:
        robot.ur_connection.set_freedrive(True, 600)
    else:
        robot.ur_connection.set_freedrive(None)

def reset_orientation() -> None:
    """Reset robot orientation to predefined pose."""
    pos = robot.ur_connection.getl()
    pos[3] = math.pi / 2
    pos[4] = 0
    pos[5] = 0
    robot.ur_connection.movel(pos, acc=1.0, vel=1.0)

def adjust_orientation_yaw() -> None:
    """Adjust only the yaw of the end-effector."""
    pos = robot.ur_connection.getl()
    pos[3] = math.pi / 2
    robot.ur_connection.movel(pos, acc=1.0, vel=0.5)

def on_press(key) -> None:
    """Handle key press events."""
    global joint_mode
    try:
        if key == Key.up:
            move_robot(1, 1)
        elif key == Key.down:
            move_robot(1, -1)
        elif key == Key.right:
            move_robot(0, -1)
        elif key == Key.left:
            move_robot(0, 1)
        elif key.char == 'w':
            move_robot(2, 1)
        elif key.char == 's':
            move_robot(2, -1)
        elif key.char == 'q':
            move_robot(5, 1)
        elif key.char == 'e':
            move_robot(5, -1)
        elif key.char == '4':
            move_robot(4, 1)
        elif key.char == '6':
            move_robot(4, -1)
        elif key.char == '7':
            move_robot(3, 1)
        elif key.char == '9':
            move_robot(3, -1)
        elif key.char == 'j':
            joint_mode = not joint_mode  # Toggle movej/movel mode
            print(f"Joint mode: {'ON' if joint_mode else 'OFF'}")
        elif key.char == 'l':
            reset_orientation()
        elif key.char == 'v':
            adjust_orientation_yaw()
        elif key.char == 'f':
            toggle_freedrive()
    except AttributeError:
        pass

def on_release(key) -> None:
    """Handle key release events."""
    try:
        if key in {Key.up, Key.down, Key.right, Key.left} or (hasattr(key, 'char') and key.char in {'w', 's', '8', '2', '4', '6', '7', '9'}):
            move_robot(0, 0)
            move_robot(1, 0)
            move_robot(2, 0)
            move_robot(3, 0)
            move_robot(4, 0)
            move_robot(5, 0)
    except AttributeError:
        pass

# Keyboard Listener
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()

# Main Loop
while not exit_program:
    if not freedrive_mode:
        if joint_mode:
            pass  # In joint mode, move only when keys are pressed
        else:
            robot.ur_connection.speedl_tool(velocity, acc=1.0, time=0.1)
    time.sleep(0.05)
