from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_explainer',
            executable='navigation_explainer_srv',
            output='screen'),
    ])