# UR Robot Keyboard Control

This script allows you to control a Universal Robots (UR) robotic arm using a keyboard.

- Default movement: **Cartesian linear velocity** (via `speedl_tool`)
- Switch to **joint movement** mode (`movej`) by pressing `j`
- Support for Freedrive Mode

---

## Setup

### Requirements
- Python 3.7+
- `pynput` library
- UR Interface SDK (`ur_interface`)

### Usage
```bash
python ur_keyboard_control.py --url <robot_ip> --lin_speed <linear_speed> --rad_speed <radial_speed>
```

Defaults:
- `--url 146.137.240.38`
- `--lin_speed 0.01` (m/s)
- `--rad_speed 0.01` (rad/s)

---

## Keyboard Mappings

| Key        | Action                                      | Mode         |
|------------|---------------------------------------------|--------------|
| Up Arrow   | Move +Y                                     | Linear       |
| Down Arrow | Move -Y                                     | Linear       |
| Left Arrow | Move +X                                     | Linear       |
| Right Arrow| Move -X                                     | Linear       |
| `w`        | Move +Z (up)                                | Linear       |
| `s`        | Move -Z (down)                              | Linear       |
| `q`        | Rotate +Yaw (around Z axis)                 | Linear       |
| `e`        | Rotate -Yaw (around Z axis)                 | Linear       |
| `4`        | Rotate +Pitch (around Y axis)               | Linear       |
| `6`        | Rotate -Pitch (around Y axis)               | Linear       |
| `7`        | Rotate +Roll (around X axis)                | Linear       |
| `9`        | Rotate -Roll (around X axis)                | Linear       |
| `j`        | Toggle between Cartesian (linear) and Joint mode | Switch   |
| `l`        | Reset orientation                          | Action       |
| `v`        | Adjust yaw only                            | Action       |
| `f`        | Toggle Freedrive Mode                      | Switch       |

---

## Behavior Details
- **Default**: Move robot via `speedl_tool(velocity)`.
- **Joint Mode (`j`)**: Adjust joints individually using the same keys.
- **Freedrive Mode (`f`)**: Manual physical movement allowed.
- **Reset Orientation (`l`)**: Resets TCP orientation to face forward.
- **Adjust Yaw (`v`)**: Correct yaw angle only.

---

## Notes
- Always monitor robot movement closely to avoid collisions.
- Speed settings are user adjustable via CLI arguments.
- Code is built to be extendable and easily modifiable.

---