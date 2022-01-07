from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='remote_pubsub',
            executable='talker',
            name='talker'
        ),
        Node(
            package='remote_pubsub',
            executable='listener',
            name='listener1'
        ),
    ])
