from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="py_custom",
                executable="transformation_server",
                name="transformation_server",
            ),
            Node(
                package="py_custom",
                executable="transformation_client",
                name="transformation_client",
            ),
        ]
    )
