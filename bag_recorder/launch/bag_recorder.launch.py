
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bag_recorder',
            executable='bag_recorder_srv',
            output='screen'),
    ])

