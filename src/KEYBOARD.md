# UR Robot Keyboard Control

This script allows you to control a Universal Robots (UR) robotic arm using a keyboard.
It mimics teach pendant behavior with precise step-based movements using TCP or joint control.

---

## Features

- **Step-based Cartesian movement** using `movel()`
- **Joint control** using `movej()`
- **Freedrive toggle** for manual movement
- **Real-time printout of pose and joint angles**
- **Orientation control** (Rx, Ry, Rz) from keyboard
- **Set gripper perpendicular to ground instantly**
- Highly responsive and configurable via command-line arguments

---

## Setup

### Requirements
- Python 3.7+
- `pynput` library
- UR Interface SDK (`ur_interface`)

### Installation
If `ur_module` package is installed, you can skip this install step.
```bash
pip install pynput
# And make sure `ur_interface` is installed in your environment
```

### Usage
```bash
python keyboard_control.py \
    --url <robot_ip> \
    --step <step_size_meters_or_radians> \
    --joint_step <joint_step_radians> \
    --speed <velocity_and_acceleration>
```

Defaults:
- `--url 146.137.240.38`
- `--step 0.01` (m)
- `--joint_step 0.05` (rad)
- `--speed 2.0` (max linear/angular speed)

---

## Keyboard Mappings

### Cartesian (TCP) Movement:
| Key        | Action                      |
|------------|-----------------------------|
| Up Arrow   | +Y (forward)                |
| Down Arrow | -Y (backward)               |
| Left Arrow | +X (left)                   |
| Right Arrow| -X (right)                  |
| `w`        | +Z (up)                     |
| `s`        | -Z (down)                   |

### Orientation (TCP rotation):
| Key        | Action                      |
|------------|-----------------------------|
| `u`        | +Roll (Rx)                  |
| `o`        | -Roll (Rx)                  |
| `m`        | +Pitch (Ry)                 |
| `,`        | -Pitch (Ry)                 |
| `j`        | +Yaw (Rz)                   |
| `l`        | -Yaw (Rz)                   |

### Joint Rotation:
| Key        | Joint Index | Direction     |
|------------|-------------|---------------|
| `1` / `2`  | Joint 0     | + / -          |
| `3` / `4`  | Joint 1     | + / -          |
| `5` / `6`  | Joint 2     | + / -          |
| `7` / `8`  | Joint 3     | + / -          |
| `9` / `0`  | Joint 4     | + / -          |
| `-` / `=`  | Joint 5     | + / -          |

### Functional Keys:
| Key        | Action                                     |
|------------|--------------------------------------------|
| `f`        | Enable Freedrive mode                      |
| `p`        | Print current pose and joint angles        |
| `g`        | Set gripper to face straight down (Rx=pi)  |
| `ESC`      | Quit                                        |

---

## Notes
- This tool is useful for teaching poses and validating motion steps.
- Ideal for position tuning, pick-and-place prep, and lab robotics.
- Modify step sizes and speed for precision or fast setup.
- Extendable and safe to use in supervised testing environments.

---
