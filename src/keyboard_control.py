"""
UR Robot Keyboard Step Control

Mimics teach pendant behavior: each key press moves the robot a small step using `movel()` for translation/orientation,
and now supports rotating joints one by one with dedicated keys.
Includes basic error recovery for 'robot stopped' errors.
Now tuned for maximum speed and responsiveness.
"""

import argparse
import math
import sys
import termios
import tty

from urx.urrobot import RobotException

from ur_interface.ur import UR


def get_key():
    """Get a key including arrow keys"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        char = sys.stdin.read(1)
        # Handle arrow keys (escape sequences)
        if char == "\x1b":  # ESC sequence
            char += sys.stdin.read(2)
            if char == "\x1b[A":
                return "UP"
            elif char == "\x1b[B":
                return "DOWN"
            elif char == "\x1b[C":
                return "RIGHT"
            elif char == "\x1b[D":
                return "LEFT"
            else:
                return "ESC"
        return char
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return char


parser = argparse.ArgumentParser(description="Terminal-based UR Robot Control")
parser.add_argument("-u", "--url", default="146.137.240.38", help="UR robot IP address")
parser.add_argument("-s", "--step", default=0.01, type=float, help="Step size in meters or radians")
parser.add_argument("-j", "--joint_step", default=0.05, type=float, help="Step size for joint rotations in radians")
parser.add_argument("--speed", default=0.5, type=float, help="Speed for movements (m/s or rad/s)")
parser.add_argument("-tcp", default=[0,0,0,0,0,0], type=list, help="Set tcp settings in rad")
parser.add_argument("-base", default=[0,0,0,0,0,0], type=list, help="Set base reference frame")
args = parser.parse_args()
print(args.base)
robot = UR(args.url,tcp_pose=args.tcp, base_reference_frame=args.base)
step = args.step
if step >= 0.1:
    print("Warning: Step size is large, may cause abrupt movements! Setting to 0.1")
    step = 0.1  # Cap step size to prevent large jumps
joint_step = args.joint_step
speed = args.speed


def step_move(dx=0, dy=0, dz=0, rx=0, ry=0, rz=0):
    """Move the robot in a specified direction."""
    try:
        pos = robot.ur_connection.getl()
        pos[0] += dx
        pos[1] += dy
        pos[2] += dz
        pos[3] += rx
        pos[4] += ry
        pos[5] += rz
        robot.ur_connection.movel(pos, acc=speed, vel=speed)
        return True
    except RobotException as e:
        print(f"[RobotException] {e}")
        print("Attempting to resume from protective stop...")
        robot.ur_dashboard.unlock_protective_stop()
        return False
    except Exception as e:
        print(f"[ERROR] {e}")
        return False


def step_joint(joint_idx=0, sign=1):
    """Rotate a specific joint by a small step."""
    try:
        joints = robot.ur_connection.getj()
        joints[joint_idx] += sign * joint_step
        robot.ur_connection.movej(joints, acc=speed, vel=speed)
        return True
    except RobotException as e:
        print(f"[RobotException] {e}")
        robot.ur_dashboard.unlock_protective_stop()
        return False
    except Exception as e:
        print(f"[ERROR] {e}")
        return False


def print_status():
    """Print current robot status"""
    try:
        tcp = robot.ur_connection.getl()
        joints = robot.ur_connection.getj()
        print(f"\nTCP Pose: {[round(p, 6) for p in tcp]}")
        print(f"Joints: {[round(j, 6) for j in joints]}")
    except Exception as e:
        print(f"[ERROR] {e}")


def print_help():
    """Print control instructions"""
    print("\n" + "=" * 50)
    print("UR ROBOT TERMINAL CONTROL")
    print("=" * 50)
    print("Movement Controls:")
    print("  Arrow keys = X/Y movement")
    print("  W/S = Z-axis up/down")
    print("  U/O = Roll (Rx), J/L = Yaw (Rz), M/, = Pitch (Ry)")
    print("\nJoint Controls:")
    print("  1/2 = Joint 1, 3/4 = Joint 2, 5/6 = Joint 3")
    print("  7/8 = Joint 4, 9/0 = Joint 5, -/= = Joint 6")
    print("\nCommands:")
    print("  ? = Show this help")
    print("  P = Print current position")
    print("  G = Face gripper down")
    print("  F = Freedrive mode")
    print("  ESC = Quit")
    print("=" * 50)


def main():
    """Main function to run the keyboard control loop"""
    print_help()

    try:
        while True:
            print("\nPress key (? for help, ESC to quit): ", end="", flush=True)
            key = get_key()

            if key == "ESC":
                print("ESC")
                break
            elif len(key) == 1:
                print(key)  # Echo single characters
            else:
                print(key)  # Echo arrow keys

            action = ""
            success = True

            # Arrow key movement (exactly like original)
            if key == "UP":
                success = step_move(dy=step)
                action = "Move +Y (Up)"
            elif key == "DOWN":
                success = step_move(dy=-step)
                action = "Move -Y (Down)"
            elif key == "LEFT":
                success = step_move(dx=step)
                action = "Move +X (Left)"
            elif key == "RIGHT":
                success = step_move(dx=-step)
                action = "Move -X (Right)"

            # Character controls (exactly like original)
            elif key == "w":
                success = step_move(dz=step)
                action = "Move +Z"
            elif key == "s":
                success = step_move(dz=-step)
                action = "Move -Z"
            elif key == "j":
                success = step_move(rz=step)
                action = "Rotate +Rz (Yaw)"
            elif key == "l":
                success = step_move(rz=-step)
                action = "Rotate -Rz (Yaw)"
            elif key == "u":
                success = step_move(rx=step)
                action = "Rotate +Rx (Roll)"
            elif key == "o":
                success = step_move(rx=-step)
                action = "Rotate -Rx (Roll)"
            elif key == "m":
                success = step_move(ry=step)
                action = "Rotate +Ry (Pitch)"
            elif key == ",":
                success = step_move(ry=-step)
                action = "Rotate -Ry (Pitch)"

            # Joint controls (exactly like original)
            elif key == "1":
                success = step_joint(0, 1)
                action = "Joint 1 +"
            elif key == "2":
                success = step_joint(0, -1)
                action = "Joint 1 -"
            elif key == "3":
                success = step_joint(1, 1)
                action = "Joint 2 +"
            elif key == "4":
                success = step_joint(1, -1)
                action = "Joint 2 -"
            elif key == "5":
                success = step_joint(2, 1)
                action = "Joint 3 +"
            elif key == "6":
                success = step_joint(2, -1)
                action = "Joint 3 -"
            elif key == "7":
                success = step_joint(3, 1)
                action = "Joint 4 +"
            elif key == "8":
                success = step_joint(3, -1)
                action = "Joint 4 -"
            elif key == "9":
                success = step_joint(4, 1)
                action = "Joint 5 +"
            elif key == "0":
                success = step_joint(4, -1)
                action = "Joint 5 -"
            elif key == "-":
                success = step_joint(5, 1)
                action = "Joint 6 +"
            elif key == "=":
                success = step_joint(5, -1)
                action = "Joint 6 -"

            # Special commands (exactly like original)
            elif key == "p":
                print_status()
                continue
            elif key == "g":
                try:
                    pos = robot.ur_connection.getl()
                    pos[3] = math.pi
                    pos[4] = 0
                    pos[5] = 0
                    robot.ur_connection.movel(pos, acc=speed, vel=speed)
                    action = "Gripper down"
                except Exception as e:
                    print(f"[ERROR] {e}")
                    success = False
            elif key == "f":
                try:
                    robot.ur_connection.set_freedrive(True, 600)
                    action = "Freedrive ON"
                except Exception as e:
                    print(f"[ERROR] {e}")
                    success = False
            elif key == "?":
                print_help()
                continue
            elif key == "\x03":  # Ctrl+C
                break
            else:
                print(f"Unknown command: {key}")
                continue

            # Show result
            if success:
                print(f"✓ {action}")
            else:
                print(f"✗ {action} - FAILED")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping robot and exiting...")
        try:
            robot.ur_connection.stopl(2)
        except Exception as e:
            print(f"[ERROR] Failed to stop robot: {e}")


if __name__ == "__main__":
    main()
