
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
    tool = LaunchConfiguration("tool")
    tool_address = LaunchConfiguration("tool_address")
    
    declare_use_ur_ip_cmd = DeclareLaunchArgument(
        name = "ip",
        default_value= "146.137.240.38",
        description= "Flag to accept UR IP"
        )
    
    declare_use_ur_tool_cmd = DeclareLaunchArgument(
        name = "tool",
        default_value= "None",
        description= "Flag to accept UR tool"
        )
    
    declare_use_ur_tool_address_cmd = DeclareLaunchArgument(
        name = "tool_address",
        default_value= "None",
        description= "Flag to accept UR tool address"
        )
    
    ur5_client = Node(
            package = 'ur5_client',
            namespace = 'ur5_client',
            executable = 'ur5_client',
            output = "screen",
            name='UR5_Client_Node',
            parameters = [{"ip":ip},{"tool":tool},{"tool_address":tool_address}],
            emulate_tty=True
    )
    launch_d.add_action(declare_use_ur_ip_cmd)
    launch_d.add_action(declare_use_ur_tool_cmd)
    launch_d.add_action(declare_use_ur_tool_address_cmd)

    launch_d.add_action(ur5_client)

    return launch_d