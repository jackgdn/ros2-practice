import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    # 初始化FibonacciActionServer节点
    def __init__(self):
        super().__init__("fibonacci_action_server")
        self.__action_server = ActionServer(
            self, Fibonacci, "fibonacci", self.__execute
        )

    async def __execute(self, goal_handle):
        # 执行Fibonacci序列生成的逻辑
        self.get_logger().info("Executing goal...")
        sequence = [0, 1]

        if goal_handle.request.order < 0:
            # 如果请求的Fibonacci序列的阶数小于0，记录错误并中止目标
            self.get_logger().error("Invalid order, aborting goal...")
            goal_handle.abort()
            result = Fibonacci.Result()
            result.sequence = sequence
            return result

        for i in range(1, goal_handle.request.order):
            # 检查目标是否被取消，如果是则记录并返回当前结果
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled.")
                result = Fibonacci.Result()
                result.sequence = sequence
                return result

            # 生成Fibonacci序列
            sequence.append(sequence[i] + sequence[i - 1])

            # 发布反馈信息
            feedback = Fibonacci.Feedback()
            feedback.partial_sequence = sequence
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Sending feedback: {sequence}")
            rclpy.spin_once(self, timeout_sec=0.1)

        # 目标成功完成
        goal_handle.succeed()

        # 设置并返回结果
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info(f"Result: {sequence}")
        return result


def main(args=None):
    # 主函数，初始化ROS2节点并运行ActionServer
    rclpy.init(args=args)
    rclpy.spin(FibonacciActionServer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
