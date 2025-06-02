from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="py_custom",
                executable="coordinate_publisher",
                name="coordinate_publisher",
            ),
            Node(
                package="py_custom",
                executable="coordinate_subscriber",
                name="coordinate_subscriber",
            ),
        ]
    )
