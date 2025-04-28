"""Methods for calculating the forward kinematics of UR robots."""

import numpy as np


def _load_dh_parameters(model: str) -> dict:
    """Load Denavit-Hartenberg parameters for the specified robot model."""
    models = {
        "UR3E": {
            "a": [0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0],
            "d": [0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819],
            "alpha": [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0],
        },
        "UR5E": {
            "a": [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0],
            "d": [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996],
            "alpha": [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0],
        },
        "UR10E": {
            "a": [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0],
            "d": [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655],
            "alpha": [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0],
        },
        "UR16E": {
            "a": [0.0, -0.271, -0.36435, 0.0, 0.0, 0.0],
            "d": [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655],
            "alpha": [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0],
        },
    }
    if model not in models:
        raise ValueError(f"Unsupported robot model: {model}")
    return models[model]


def _dh_transform(a, alpha, d, theta):
    """Calculate the Denavit-Hartenberg transformation matrix."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)

    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]])


def forward_kinematics(joints: list, dh_params) -> np.ndarray:
    """Calculate the forward kinematics for the given joint angles."""
    if len(joints) != 6:
        raise ValueError("Expected 6 joint angles!")

    T = np.eye(4)
    for i in range(6):
        a = dh_params["a"][i]
        d = dh_params["d"][i]
        alpha = dh_params["alpha"][i]
        theta = joints[i]

        T_i = _dh_transform(a, alpha, d, theta)
        T = T @ T_i

    return T


def pose_from_matrix(T: np.ndarray) -> list:
    """Convert a transformation matrix to a pose (position and rotation)."""
    pos = T[:3, 3]
    rot = rotation_to_axis_angle(T[:3, :3])
    return [float(pos[0]), float(pos[1]), float(pos[2]), float(rot[0]), float(rot[1]), float(rot[2])]


def rotation_to_axis_angle(R: np.ndarray) -> list:
    """Convert a rotation matrix to axis-angle representation."""
    angle = np.arccos((np.trace(R) - 1) / 2)
    if np.isclose(angle, 0):
        return [0.0, 0.0, 0.0]
    rx = (R[2, 1] - R[1, 2]) / (2 * np.sin(angle))
    ry = (R[0, 2] - R[2, 0]) / (2 * np.sin(angle))
    rz = (R[1, 0] - R[0, 1]) / (2 * np.sin(angle))
    return [float(rx * angle), float(ry * angle), float(rz * angle)]


def get_pose_from_joint_angles(joints: list, robot_model="UR5e") -> list:
    """Get the pose from joint angles for the specified robot model."""
    dh_parameters = _load_dh_parameters(robot_model.upper())
    T = forward_kinematics(joints, dh_params=dh_parameters)
    pose = pose_from_matrix(T)
    return pose
