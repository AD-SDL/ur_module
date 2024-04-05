# UR Module

## Overview

The `ur_module` repository is a comprehensive package that includes a UR driver and a REST node for Universal Robots (UR). This module is designed to provide a robust and flexible interface for controlling UR robots, offering a variety of control interfaces for different use cases. The repository supports initialization via the UR dashboard, motion control using URx, and end-effector management.

## Features

### UR Driver Package

The UR driver package in this repository encompasses various remote interfaces to enable comprehensive control over the UR robot. The supported interfaces include:

1. **Initialization via UR Dashboard:**
   - Seamless integration with the UR dashboard for easy initialization.
   - Dashboard Server features include loading and playing URP programs, power on and off, querying robot status, and setting operational modes.

2. **Motion Control using URx:**
   - URx features provide precise control over robot motion.

3. **End-Effector Management:**
   - Support for a variety of end-tools, including:
     - Robotiq Gripper (Modbus TCP/IP)
     - TriContenent Pipette (RS485)
     - Robotiq Screwdriver (Interpreter Socket)
     - Robotiq Vacuum Gripper (Interpreter Socket)
     - OpenTrone Pipette (PyEpics)
     - ATI Tool Changer (PyEpics)

### Dashboard Server Features

Seamless integration with the UR dashboard for easy initialization. The dashboard server facilitates the following operations:

- Load and play URP programs.
- Power on and off the robot.
- Query the robot status.
- Set the operational mode.

### URx Features

URx features provide fine-grained control over robot motion.

### End-Tools Integration

Support for a variety of end-tools, including communication protocols such as TCP/IP, RS485, Interpreter Socket, and PyEpics.

- **Robotiq Gripper:** TCP/IP
- **TriContenent Pipette:** RS485
- **Robotiq Screwdriver:** Interpreter Socket
- **Robotiq Vacuum Gripper:** Interpreter Socket
- **OpenTrone Pipette:** PyEpics
- **ATI Tool Changer:** PyEpics

### AI Camera Integration

The repository includes AI camera support for real-time object detection and recognition. The detected target locations can be seamlessly transitioned to the UR robot for further action.

## REST Node

The role of the REST node in this repository is to establish communication between UR robots and the higher-level system of WEI (Workflow Execution Interface). This node acts as a bridge, enabling seamless integration of UR robots into a broader robotic ecosystem.

## Installation

<!-- To use the `ur_module` repository, follow the installation instructions provided in the [Installation Guide](/docs/installation.md). -->

```
mkdir ~/wei_ws/src
cd ~/wei_ws/src
git clone https://github.com/AD-SDL/ur_module.git
cd ~/wei_ws
colcon build
source install/setup.bash
```
## Usage

Detailed information on how to use the UR driver and ROS node is available in the [User Guide](/docs/user_guide.md).

## Contribution

Contributions are welcome! If you have ideas for improvements or find any issues, please open an issue or submit a pull request.

## License

This repository is licensed under the MIT License - see the [LICENSE](/LICENSE) file for details.

---
