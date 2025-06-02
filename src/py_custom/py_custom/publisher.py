from random import uniform

import rclpy
from rclpy.node import Node

from custom_interfaces.msg import Coordinate


class CoordinatePublisher(Node):

    def __init__(self):
        super().__init__("coordinate_publisher")
        self.__publisher = self.create_publisher(Coordinate, "coordinate", 10)
        self.__timer = self.create_timer(2, self.__callback)

    def __callback(self):
        message = Coordinate()
        message.x = uniform(-180, 180)
        message.y = uniform(-180, 180)
        self.__publisher.publish(message)
        self.get_logger().info(f"Publishing: ({message.x}, {message.y})")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CoordinatePublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
