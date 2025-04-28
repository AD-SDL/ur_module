# File: ur_interface/ur_calibrated_kinematics.py
import numpy as np

class URCalibratedKinematics:
    def __init__(self, robot_model: str = "UR5e"):
        self.robot_model = robot_model.upper()
        self.dh_params = self._load_dh_parameters(self.robot_model)

    def _load_dh_parameters(self, model: str) -> dict:
        models = {
            "UR3E": {
                "a": [0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0],
                "d": [0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819],
                "alpha": [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
            },
            "UR5E": {
                "a": [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0],
                "d": [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996],
                "alpha": [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
            },
            "UR10E": {
                "a": [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0],
                "d": [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655],
                "alpha": [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
            },
            "UR16E": {
                "a": [0.0, -0.271, -0.36435, 0.0, 0.0, 0.0],
                "d": [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655],
                "alpha": [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
            }
        }
        if model not in models:
            raise ValueError(f"Unsupported robot model: {model}")
        return models[model]

    def _dh_transform(self, a, alpha, d, theta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)

        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joints: list) -> np.ndarray:
        if len(joints) != 6:
            raise ValueError("Expected 6 joint angles!")

        T = np.eye(4)
        for i in range(6):
            a = self.dh_params["a"][i]
            d = self.dh_params["d"][i]
            alpha = self.dh_params["alpha"][i]
            theta = joints[i]

            T_i = self._dh_transform(a, alpha, d, theta)
            T = T @ T_i

        return T

    def pose_from_matrix(self, T: np.ndarray) -> list:
        pos = T[:3, 3]
        rot = self.rotation_to_axis_angle(T[:3, :3])
        return [float(pos[0]), float(pos[1]), float(pos[2]), float(rot[0]), float(rot[1]), float(rot[2])]

    def rotation_to_axis_angle(self, R: np.ndarray) -> list:
        angle = np.arccos((np.trace(R) - 1) / 2)
        if np.isclose(angle, 0):
            return [0.0, 0.0, 0.0]
        rx = (R[2,1] - R[1,2]) / (2*np.sin(angle))
        ry = (R[0,2] - R[2,0]) / (2*np.sin(angle))
        rz = (R[1,0] - R[0,1]) / (2*np.sin(angle))
        return [float(rx * angle), float(ry * angle), float(rz * angle)]


    def get_pose_from_joint_angles(self, joints: list) -> list:
        T = self.forward_kinematics(joints)
        pose = self.pose_from_matrix(T)
        return pose

# Example Usage
if __name__ == "__main__":
    from urx import Robot
    robot = Robot("146.137.240.38")
    robot.set_tcp((0, 0, 0, 0, 0, 0))
    # robot.get_tcp_force()
    joints = robot.getj()
    print(robot.getl())
    robot.close()
    kinematics = URCalibratedKinematics()
    pose = kinematics.get_pose_from_joint_angles(joints)
    print("✅ TCP Pose for movel:", pose)
