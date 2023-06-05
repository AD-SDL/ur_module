
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

def generate_launch_description():

    launch_d = LaunchDescription()

    ip = LaunchConfiguration("ip")
    gripper = LaunchConfiguration("gripper")
    vacuum_gripper = LaunchConfiguration("vacuum_gripper")
    screwdriver = LaunchConfiguration("screwdriver")
    tool_changer_pv = LaunchConfiguration("tool_changer_pv")
    pipette_pv = LaunchConfiguration("pipette_pv")
    camera_pv = LaunchConfiguration("camera_pv")

    
    declare_use_ur_ip_cmd = DeclareLaunchArgument(
        name = "ip",
        default_value= "146.137.240.38",
        description= "Flag to accept UR IP"
        )
    
    declare_use_ur_gripper_cmd = DeclareLaunchArgument(
        name = "gripper",
        default_value= "False",
        description= "Flag to accept UR gripper"
        )
    
    declare_use_ur_vacuum_gripper_cmd = DeclareLaunchArgument(
        name = "vacuum_gripper",
        default_value= "False",
        description= "Flag to accept UR vacuum gripper"
        )
    
    declare_use_ur_screwdriver_cmd = DeclareLaunchArgument(
        name = "screwdriver",
        default_value= "False",
        description= "Flag to accept UR screwdriver"
        )
    declare_use_ur_tool_changer_pv_cmd = DeclareLaunchArgument(
        name = "tool_changer_pv",
        default_value= "None",
        description= "Flag to accept UR tool address"
        )
    
    declare_use_ur_pipette_pv_cmd = DeclareLaunchArgument(
        name = "pipette_pv",
        default_value= "None",
        description= "Flag to accept UR pipette_pv"
        )
    
    declare_use_ur_camera_pv_cmd = DeclareLaunchArgument(
        name = "camera_pv",
        default_value= "None",
        description= "Flag to accept UR camera_pv"
        )
    
    ur5_client = Node(
            package = 'ur_client',
            namespace = 'std_ns',
            executable = 'ur_client',
            output = "screen",
            name='UR5_Client_Node',
            parameters = [{"ip":ip},
                          {"gripper":gripper},
                          {"vacuum_gripper":vacuum_gripper},
                          {"screwdriver":screwdriver},
                          {"tool_changer_pv":tool_changer_pv},
                          {"pipette_pv":pipette_pv},
                          {"camera_pv":camera_pv}],
            emulate_tty=True
    )
    launch_d.add_action(declare_use_ur_ip_cmd)
    launch_d.add_action(declare_use_ur_gripper_cmd)
    launch_d.add_action(declare_use_ur_vacuum_gripper_cmd)
    launch_d.add_action(declare_use_ur_screwdriver_cmd)
    launch_d.add_action(declare_use_ur_tool_changer_pv_cmd)
    launch_d.add_action(declare_use_ur_pipette_pv_cmd)
    launch_d.add_action(declare_use_ur_camera_pv_cmd)
    launch_d.add_action(ur5_client)

    return launch_d