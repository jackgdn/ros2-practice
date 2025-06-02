import math

import rclpy
from rclpy.node import Node

from custom_interfaces.srv import CoordinateTransformation


class TransformationServer(Node):

    def __init__(self):
        super().__init__("transformation_server")
        self.__service = self.create_service(
            CoordinateTransformation, "transformation", self.__handle
        )

    def __handle(self, request, response):
        x = request.x
        y = request.y
        self.get_logger().info(f"Received request: x={x} y={y}")
        response.radius = math.sqrt(x**2 + y**2)
        response.theta = math.atan2(y, x)
        return response


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TransformationServer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
