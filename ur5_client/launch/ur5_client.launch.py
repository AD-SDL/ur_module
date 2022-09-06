
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [FindPackageShare("ur5_client"), "config", "ur5_move_config.yaml"]
    )
    launch_d = LaunchDescription()
    
    ur5_client = Node(
            package = 'ur5_client',
            namespace = 'ur5_client',
            executable = 'ur5_client',
            output = "screen",
            name='ur5_Node'
    )

    launch_d.add_action(ur5_client)
    return launch_d