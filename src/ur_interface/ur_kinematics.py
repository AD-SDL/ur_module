"""Forward kinematics for UR e-Series robots."""

import numpy as np
from scipy.spatial.transform import Rotation as R

# DH Parameters for different UR e-Series robots
DH_PARAMETERS = {
    "ur3e": {
        "d": [0.1519, 0, 0, 0.11235, 0.08535, 0.0819],
        "a": [0, -0.24365, -0.21325, 0, 0, 0],
        "alpha": [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0],
    },
    "ur5e": {
        "d": [0.1625, 0, 0, 0.1333, 0.0997, 0.0996],
        "a": [0, -0.425, -0.3922, 0, 0, 0],
        "alpha": [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0],
    },
    "ur16e": {
        "d": [0.1807, 0, 0, 0.17415, 0.11985, 0.11655],
        "a": [0, -0.505, -0.352, 0, 0, 0],
        "alpha": [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0],
    },
}


def joints_to_pose_vector(joint_angles_deg, robot_model="ur5e"):
    """
    Convert 6 joint angles (degrees) into a pose vector [x, y, z, rx, ry, rz]
    compatible with URX's movel function.

    Args:
        joint_angles_deg (list or array): 6 joint angles in degrees.
        robot_model (str): Robot model ("ur3e", "ur5e", or "ur16e").

    Returns:
        list: Pose vector [x, y, z, rx, ry, rz] with standard Python floats.
    """
    if len(joint_angles_deg) != 6:
        raise ValueError("Exactly 6 joint angles must be provided.")

    if robot_model not in DH_PARAMETERS:
        raise ValueError(f"Unsupported robot model '{robot_model}'. Supported models: {list(DH_PARAMETERS.keys())}")

    params = DH_PARAMETERS[robot_model]
    d = params["d"]
    a = params["a"]
    alpha = params["alpha"]

    joint_angles_rad = np.radians(joint_angles_deg)

    T = np.eye(4)
    for i in range(6):
        theta = joint_angles_rad[i]
        T = T @ _dh_transform(a[i], alpha[i], d[i], theta)

    position = T[:3, 3]
    rotation_matrix = T[:3, :3]

    r = R.from_matrix(rotation_matrix)
    rotvec = r.as_rotvec()

    pose_vector = [float(x) for x in position] + [float(x) for x in rotvec]
    return pose_vector


def _dh_transform(a, alpha, d, theta):
    """Compute individual DH transformation matrix."""
    sa, ca = np.sin(alpha), np.cos(alpha)
    st, ct = np.sin(theta), np.cos(theta)

    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]])


def is_joint_angles(values):
    """Check if the given values represent joint angles or a pose."""
    if len(values) != 6:
        return False

    position = values[:3]
    rotation = values[3:]  # Noqa

    # If any position value > 2.0 meters, it must be a pose
    if any(abs(p) > 2.0 for p in position):
        return False

    # If all values are smaller than 2pi, treat as joint angles
    if all(abs(v) <= 2 * np.pi for v in values):
        # If position values are very small (typical for angles), it's likely joints
        if all(abs(p) < 1.5 for p in position):
            return True

    return False


if __name__ == "__main__":
    """ Example usage of the joints_to_pose_vector function. """
    example_joint_angles = [
        0.2069321870803833,
        -1.4558529642275353,
        -1.5868407487869263,
        -1.665375371972555,
        1.5850671529769897,
        -1.350588623677389,
    ]

    print("UR3e Pose:")
    pose_ur3e = joints_to_pose_vector(example_joint_angles, robot_model="ur3e")
    print(pose_ur3e)

    print("\nUR5e Pose:")
    pose_ur5e = joints_to_pose_vector(example_joint_angles, robot_model="ur5e")
    print(pose_ur5e)

    print("\nUR16e Pose:")
    pose_ur16e = joints_to_pose_vector(example_joint_angles, robot_model="ur16e")
    print(pose_ur16e)
