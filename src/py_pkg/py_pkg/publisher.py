import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):  # 继承 Node 类，表示这是一个 ROS2 节点

    def __init__(self):
        super().__init__("minimal_publisher")  # 初始化节点，命名为 minimal_publisher
        self.publisher_ = self.create_publisher(
            String, "topic", 10
        )  # 创建发布者，发布到 topic 话题，队列大小为 10
        self.timer = self.create_timer(
            0.5, self.callback
        )  # 创建定时器，每隔 0.5 秒执行一次回调函数
        self.count = 1  # 计数器

    def callback(self):
        message = String()  # 创建 String 消息
        message.data = "Hello, World: " + str(self.count)  # 设置消息内容
        self.publisher_.publish(message)  # 发布消息
        self.get_logger().info(f"Publishing: '{message.data}'")  # 打印日志
        self.count += 1


def main(args=None):
    rclpy.init(args=args)  # 初始化 rclpy
    minimal_publisher = MinimalPublisher()  # 创建节点对象
    rclpy.spin(minimal_publisher)  # 运行节点
    minimal_publisher.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭 rclpy


if __name__ == "__main__":
    main()
