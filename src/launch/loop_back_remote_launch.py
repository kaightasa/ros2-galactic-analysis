from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='loop_back',
            executable='remote_node',
            name='remote_node'
        ),
    ])
