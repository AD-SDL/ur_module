
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

    
    declare_use_ur_ip_cmd = DeclareLaunchArgument(
        name = "ip",
        default_value= "",
        description= "Flag to accept UR IP"
        )
    
    ur5_client = Node(
            package = 'ur5_client',
            namespace = 'ur5_client',
            executable = 'ur5_client',
            output = "screen",
            name='UR5_Client_Node',
            parameters = [{"peeler_port":ip}],
            emulate_tty=True
    )
    launch_d.add_action(declare_use_ur_ip_cmd)
    launch_d.add_action(ur5_client)

    return launch_d