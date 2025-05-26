import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            String, "topic", self.callback, 10
        )  # 创建订阅者，订阅 topic，队列大小为 10

    def callback(self, message):
        self.get_logger().info(f"Received: '{message.data}'")


def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS 2 上下文
    minimal_subscriber = MinimalSubscriber()  # 创建订阅者节点实例
    rclpy.spin(minimal_subscriber)  # 运行节点（阻塞式）
    minimal_subscriber.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭 ROS 2


if __name__ == "__main__":
    main()
