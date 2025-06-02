import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__("fibonacci_action_server")
        self.__action_server = ActionServer(
            self, Fibonacci, "fibonacci", self.__execute
        )

    async def __execute(self, goal_handle):
        self.get_logger().info("Executing goal...")
        sequence = [0, 1]

        if goal_handle.request.order < 0:
            self.get_logger().error("Invalid order, aborting goal...")
            goal_handle.abort()
            result = Fibonacci.Result()
            result.sequence = sequence
            return result

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled.")
                result = Fibonacci.Result()
                result.sequence = sequence
                return result

            sequence.append(sequence[i] + sequence[i - 1])

            feedback = Fibonacci.Feedback()
            feedback.partial_sequence = sequence
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Sending feedback: {sequence}")
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info(f"Result: {sequence}")
        return result


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FibonacciActionServer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
