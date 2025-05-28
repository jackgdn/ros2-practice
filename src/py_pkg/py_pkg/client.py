import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class MinimalClient(Node):
    # 初始化节点，设置客户端并启动定时器
    def __init__(self):
        super().__init__("minimal_client")
        self.client = self.create_client(AddTwoInts, "service")
        self.i = 0
        self.timer = self.create_timer(0.5, self.send_request)

    # 处理服务响应的回调函数
    def timer_callback(self, future):
        response = future.result()
        self.get_logger().info(f"Response: {response.sum}")

    # 发送请求到服务端
    def send_request(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted.")
                return
            else:
                self.get_logger().info("Service not available.")

        request = AddTwoInts.Request()
        request.a = self.i
        request.b = self.i + 1
        self.i += 2

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.timer_callback)


def main(args=None):
    # 初始化 ROS 2 客户端库
    rclpy.init(args=args)
    client = MinimalClient()
    # 运行客户端节点直到手动停止
    rclpy.spin(client)
    # 关闭 ROS 2 客户端库
    rclpy.shutdown()


if __name__ == "__main__":
    main()
