import rclpy
from rclpy.node import Node

from custom_interfaces.msg import Coordinate


class CoordinateSubscriber(Node):

    def __init__(self):
        super().__init__("coordinate_subscriber")
        self.__subscriber = self.create_subscription(
            Coordinate, "coordinate", self.__callback, 10
        )

    def __callback(self, message):
        self.get_logger().info(f"Received: ({message.x}, {message.y})")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CoordinateSubscriber())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
