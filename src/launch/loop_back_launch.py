from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='loop_back',
            executable='host_talker',
            name='host_talker'
        ),
        Node(
            package='loop_back',
            executable='remote_node',
            name='remote_node'
        ),
        Node(
            package='loop_back',
            executable='host_listener',
            name='host_listener'
        ),
    ])
