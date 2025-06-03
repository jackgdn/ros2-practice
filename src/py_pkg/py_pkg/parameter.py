import rclpy
from rclpy.node import Node


class MinimalParameterNode(Node):
    # 初始化节点并声明一个自定义参数
    def __init__(self):
        super().__init__("minimal_parameter_node")
        self.declare_parameter("custom_parameter", "Hello, World!")
        self.__timer = self.create_timer(1.0, self.__callback)

    # 定时器回调函数，用于获取并打印自定义参数的值
    def __callback(self):
        param = self.get_parameter("custom_parameter")
        self.get_logger().info(f"Custom parameter: {param.value}")


def main(args=None):
    # 初始化 ROS 2 客户端库
    rclpy.init(args=args)
    # 创建节点并开始处理事件
    rclpy.spin(MinimalParameterNode())
    # 关闭 ROS 2 客户端库
    rclpy.shutdown()


if __name__ == "__main__":
    main()
