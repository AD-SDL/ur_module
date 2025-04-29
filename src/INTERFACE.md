# UR Interface README

The `ur_interface` Python module provides a high-level programming interface for Universal Robots (UR) arms.
It abstracts motion control, tool management, and program execution for seamless integration into automated systems.

---

## Key Features

- **Robot Connection Management**: Safe connect and disconnect handling.
- **Motion Control**:
  - Cartesian movements (linear and speed control)
  - Joint space movements
- **Tool Handling**:
  - Automatic tool changing using a Wingman Tool Changer
  - Gripper control (finger gripper)
  - Pipette control (standard and Tricontinent)
  - Robotiq Screwdriver integration
- **Program Management**:
  - Upload and execute `.urp` programs
  - Droplet formation protocols
- **I/O Control**:
  - Set digital outputs to control external tools
- **Keyboard Control**:
  - Use the provided keyboard control script to manually jog the robot
  - Easily discover and save new poses or locations

---

## Installation

This project uses an **updated version of the `URX` library** that fixes several common dependency issues found in the public version.
It is critical to install the entire `ur_module` repository instead of manually installing public `urx`.

Navigate to the root directory of the repository (where `pyproject.toml` is located) and install using **PDM**:

```bash
pdm install
```

This ensures that the correct, fixed version of URX and all other dependencies are installed properly.

(madsci modules are internal and assumed to be available inside your environment.)

---

## Basic Usage Example

```python
from ur_interface.ur import UR

# Create UR object
robot = UR(hostname="192.168.0.100")

# Move to home position
home_joints = [0, -1.57, 0, -1.57, 0, 0]
robot.home(home_joints)

# Move to pick a tool
robot.pick_tool(home=home_joints, tool_loc=[list_of_tool_pose_values])

# Disconnect safely
robot.disconnect()
```

---

## Important Classes

### `Connection`
- Handles low-level TCP/IP connection to the UR robot using the URX library.
- Automatic retries if the robot is not immediately available.

### `UR`
- High-level master interface for controlling the UR robot.
- Integrates motion control, tool management, and dashboard operations.

**Major Methods Explained:**

- `home(location)`:
  - Moves the robot to a specified home location (list of 6 joint angles).

- `pick_tool(home, tool_loc, docking_axis='y', payload=0.12, tool_name=None)`:
  - Homes the robot, docks to a tool using the Wingman Tool Changer.

- `place_tool(home, tool_loc, docking_axis='y', tool_name=None)`:
  - Homes the robot, undocks and places a tool back.

- `gripper_transfer(home, source, target, ...)`:
  - Pick and place using a finger gripper between source and target positions.

- `gripper_pick(home, source, ...)`:
  - Pick an object using the finger gripper only.

- `gripper_place(home, target, ...)`:
  - Place an object at a target location using the finger gripper.

- `pipette_transfer(home, tip_loc, tip_trash, source, target, volume=10)`:
  - Complete a liquid transfer using an attached pipette.

- `run_droplet(home, tip_loc, sample_loc, droplet_loc, tip_trash)`:
  - Executes a full droplet creation protocol.

- `run_urp_program(transfer_file_path, program_name)`:
  - Uploads a `.urp` program to the UR robot controller and runs it, monitoring for completion.

---

## Tool Controllers Integrated

- **Wingman Tool Changer** (for automatic tool docking)
- **Finger Gripper Controller** (for pick and place tasks)
- **Screwdriver Controller** (for screwdriving operations)
- **Pipette Controllers** (for liquid handling tasks)

---

## Notes

- Default acceleration and velocity are set but adjustable per operation.
- Home location must be provided for safe tool picking/placing.
- Motion checks ensure safety by verifying if the robot is READY before moving.
- Error handling is included but can be expanded based on application needs.
- Example Jupyter notebooks will be provided soon to demonstrate full workflows and typical use cases.

---

