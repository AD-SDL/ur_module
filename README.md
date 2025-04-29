# UR Module

## Overview

The `ur_module` repository is a comprehensive robotics control package centered around a modernized **UR Interface**. It replaces the legacy driver system with a Python-based interface built on `madsci` infrastructure and includes tooling for motion control, tool management, and integration with external systems. It supports direct robot control through UR Dashboard and URX, and also provides keyboard-based teleoperation.

## Features

### UR Interface Package

The UR interface offers a unified API to:

1. **Initialize Robot via UR Dashboard**
   - Power on/off
   - Load and run `.urp` programs
   - Set operational modes
   - Clear faults

2. **Motion Control Using Updated URX**
   - Cartesian and joint-space control
   - Speed and acceleration tuning
   - Safe motion state checking

3. **Tool Control**
   - Support for a variety of end-tools, including:
     - Robotiq Gripper (Modbus TCP/IP)
     - TriContenent Pipette (RS485)
     - Robotiq Screwdriver (Interpreter Socket)
     - Robotiq Vacuum Gripper (Interpreter Socket)
     - OpenTrone Pipette (PyEpics)
     - ATI Tool Changer (PyEpics)

4. **I/O Digital Output Handling**
   - Set digital pins to open/close connected tools or devices

5. **Keyboard Control (Jogging)**
   - Use keyboard to jog the arm in real-time
   - Easily save poses by moving robot manually

6. **Support for Camera Integration**
   - The repository includes AI camera support for real-time object detection and recognition. The detected target locations can be seamlessly transitioned to the UR robot for further action.

## Installation

> ⚠️ This repo includes a patched and dependency-fixed version of URX. Installing this repo as a whole is critical.

```bash
cd ur_module
pdm install
```

PDM ensures all dependencies (including madsci, URX fork, etc.) are resolved in a reliable and reproducible environment.

## Usage

After installing, explore the following modules:

- `ur_interface/ur.py`: Primary control interface (`UR` class)
- `keyboard_control.py`: Manual jogging and pose discovery
- `tests/`: Example Jupyter notebooks to interact with the interface and the REST Node
   - `tests/interface_test.ipynb`
   - `tests/node_test.ipynb`

Example:
```python
from ur_interface.ur import UR
robot = UR("192.168.0.100")
robot.home([0, -1.57, 0, -1.57, 0, 0])
robot.disconnect()
```

## Contributions

We welcome pull requests and issues. Please ensure code is typed and documented.

## License

This repository is licensed under the MIT License - see the [LICENSE](/LICENSE) file for details.

