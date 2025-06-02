from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cpp_custom",
                executable="fibonacci_action_server",
                name="fibonacci_action_server",
            ),
            Node(
                package="cpp_custom",
                executable="fibonacci_action_client",
                name="fibonacci_action_client",
            ),
        ]
    )
