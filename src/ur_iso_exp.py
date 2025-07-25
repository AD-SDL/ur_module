"""
Gripper sample code for hose operation with UR robots.
OBTAIN YOUR OWN TCP AND JOINT POSITIONS BASED ON THE SETUP.
"""

from ur_interface.ur import UR

# Initialize the UR robots with their respective hostnames and base reference frames
robot5 = UR(hostname="192.168.56.101", base_reference_frame=[0, 0, 0, 45, 0, 0])  # UR5e robot arm
robot16 = UR(hostname="192.168.56.250", base_reference_frame=[0, 0, 0, 45, 0, 0])  # UR16e robot arm
# Home and source joint positions for the UR5 and UR16 arms
home_ur5 = [0.492159, -1.711406, 1.384732, 0.789635, -4.68461, -0.663519]
home_ur16 = [-0.636747, -1.457807, -1.288992, -3.830498, -1.919442, -0]
ur16_hose = [0.420687, -0.019663, 0.086713, 1.25158, 1.169632, 1.156121]
ur16_hose_return = [0.420687, -0.019663, 0.084713, 1.25158, 1.169632, 1.156121]
ur5_lock = [-0.471309, -0.142086, 0.043139, 1.462046, -0.668518, -0.664085]
docker_pos_frame1 = [0.462674, -0.191737, 0.058011, 1.255713, 1.128157, 1.141096]  # on the frame of the hose dock
docker_pos_frame2 = [0.462674, -0.191737, 0.078011, 1.255713, 1.128157, 1.141096]
docker_pos_hose1 = [0.371573, 0.401279, -0.034712, 1.600879, 0.170846, 0.22243]
docker_pos_hose2 = [0.371573, 0.401279, -0.014712, 1.600879, 0.170846, 0.22243]
# Docking self to the hose dock frame
# robot16.gripper_transfer(
#     home=home_ur16,
#     source=docker_pos_frame1,
#     source_approach_axis="z",
#     source_approach_distance=0.15,
#     target=docker_pos_hose2,
#     target_approach_axis="z",
#     target_approach_distance=0.15,
# )
# First grabbing the hose with the arm combo
# robot16.hold_hose(home=home_ur16, source=ur16_hose, source_approach_axis="-x")
# robot5.gripper_unlock_joint(
#     home=home_ur5, joint_location=ur5_lock, joint_approach_axis="y", joint_approach_distance=0.1, depth=0.006
# )
# Recover the hose and both grippers
# robot16.recover_hose(home=home_ur16, source=ur16_hose, source_approach_axis="z", source_approach_distance=0.05)
# robot5.recover_gripper(home=home_ur5, joint_location=ur5_lock, joint_approach_axis="y", joint_approach_distance=0.1)
# Place the hose back into the target dock
# robot5.gripper_unlock_joint(
#     home=home_ur5, joint_location=ur5_lock, joint_approach_axis="y", joint_approach_distance=0.1, depth=0.006
# )
# robot16.place_hose(home=home_ur16, target=ur16_hose_return, target_approach_axis="z", target_approach_distance=0.05)
# Recovering both grippers after replacing the hose
robot5.recover_gripper(home=home_ur5, joint_location=ur5_lock, joint_approach_axis="y", joint_approach_distance=0.1)
robot16.recover_gripper(
    home=home_ur16, joint_location=ur16_hose_return, joint_approach_axis="-x", joint_approach_distance=0.05
)
# Undocking self to the hose dock frame
# robot16.gripper_transfer(
#     home=home_ur16,
#     source=docker_pos_hose1,
#     source_approach_axis="z",
#     source_approach_distance=0.15,
#     target=docker_pos_frame2,
#     target_approach_axis="z",
#     target_approach_distance=0.15,
# )
# Disconnecting the robots
robot5.disconnect()
robot16.disconnect()
