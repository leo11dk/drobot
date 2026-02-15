from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drobot_ui',
            executable='ui',
            name='drobot_ui',
            output='screen',
        ),
    ])
