import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class MinimalServer(Node):
    # 初始化函数，设置节点名称并创建服务
    def __init__(self):
        super().__init__("minimal_server")
        self.service = self.create_service(AddTwoInts, "service", self.handle_add)
        self.get_logger().info("Server ready.")

    # 处理加法请求的回调函数
    def handle_add(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"Request: a={request.a}, b={request.b}")
        return response


# 主函数，初始化 ROS2 环境，创建服务节点并保持运行
def main(args=None):
    rclpy.init(args=args)
    server = MinimalServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
