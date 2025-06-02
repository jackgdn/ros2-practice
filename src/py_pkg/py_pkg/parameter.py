import rclpy
from rclpy.node import Node


class MinimalParameterNode(Node):

    def __init__(self):
        super().__init__("minimal_parameter_node")
        self.declare_parameter("custom_parameter", "Hello, World!")
        self.__timer = self.create_timer(1.0, self.__callback)

    def __callback(self):
        param = self.get_parameter("custom_parameter")
        self.get_logger().info(f"Custom parameter: {param.value}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MinimalParameterNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
