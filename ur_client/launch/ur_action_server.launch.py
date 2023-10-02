
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
    name = LaunchConfiguration("name")
    
    declare_use_ur_ip_cmd = DeclareLaunchArgument(
        name = "ip",
        default_value= "146.137.240.38",
        description= "Flag to accept UR IP"
        )
    
    declare_use_ur_name_cmd = DeclareLaunchArgument(
        name = "name",
        default_value= "ur_action_server",
        description= "Flag to accept UR node name"
        )
   
    ur_client = Node(
            package = 'ur_client',
            namespace = 'std_ns',
            executable = 'ur_action_server',
            output = "screen",
            name=name,
            parameters = [{"ip":ip}],
            emulate_tty=True
    )

    launch_d.add_action(declare_use_ur_ip_cmd)
    launch_d.add_action(declare_use_ur_name_cmd)
    launch_d.add_action(ur_client)

    return launch_d