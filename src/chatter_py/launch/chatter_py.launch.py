from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chatter_py',
            executable='pub',
            output='screen',
        ),
        Node(
            package='chatter_py',
            executable='sub',
            output='screen',
        ),
    ])
