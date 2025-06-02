from random import randint

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__("fibonacci_action_client")
        self.__action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.__goal_handle = None
        self.__resul_future = None
        self.goal_done = False

    def send_goal(self):
        if not self.__action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server unavailable.")
            self.goal_done = True
            return

        goal = Fibonacci.Goal()
        goal.order = randint(-10, 10)

        self.get_logger().info(f"Sending goal with order {goal.order}")

        self.__action_client.send_goal_async(
            goal, feedback_callback=self.__feedback_callback
        ).add_done_callback(self.__goal_response_callback)

    def __feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info(f"Received feedback: {feedback.partial_sequence}")

    def __goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            self.goal_done = True
            return

        self.get_logger().info("Goal accepted.")
        self.__goal_handle = goal_handle

        self.__goal_handle.get_result_async().add_done_callback(self.__result_callback)

    def __result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.sequence}")
        self.goal_done = True


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal()

    while not action_client.goal_done:
        rclpy.spin_once(action_client)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
