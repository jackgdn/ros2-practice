import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class MinimalServer(Node):

    def __init__(self):
        super().__init__("minimal_server")
        self.service = self.create_service(AddTwoInts, "service", self.handle_add)
        self.get_logger().info("Server ready.")

    def handle_add(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"Request: a={request.a}, b={request.b}")
        return response


def main(args=None):
    rclpy.init(args=args)
    server = MinimalServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
