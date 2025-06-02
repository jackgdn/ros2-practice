from random import uniform

import rclpy
from rclpy.node import Node

from custom_interfaces.srv import CoordinateTransformation


class TransformationClient(Node):

    def __init__(self):
        super().__init__("transformation_client")
        self.__client = self.create_client(CoordinateTransformation, "transformation")
        self.__timer = self.create_timer(2.0, self.__send_request)

    def __send_request(self):
        request = CoordinateTransformation.Request()
        request.x = uniform(-180, 180)
        request.y = uniform(-180, 180)

        self.future = self.__client.call_async(request)
        self.future.add_done_callback(self.__callback)

    def __callback(self, future):
        self.get_logger().info(
            f"Received response: radius={future.result().radius} theta={future.result().theta}"
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TransformationClient())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
