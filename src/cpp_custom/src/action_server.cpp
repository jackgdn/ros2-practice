#include "custom_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using custom_interfaces::action::Fibonacci;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

// 斐波那契数列动作服务器类，继承自rclcpp::Node
class FibonacciActionServer : public rclcpp::Node {
  public:
    // 构造函数，初始化节点并创建动作服务器
    FibonacciActionServer() : Node("fibonacci_action_server") {
        this->action_server_ =
            rclcpp_action::create_server<Fibonacci>(this,
                                                    "fibonacci",
                                                    std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
                                                    std::bind(&FibonacciActionServer::handle_cancel, this, _1),
                                                    std::bind(&FibonacciActionServer::handle_accepted, this, _1));
    }

  private:
    // 动作服务器的共享指针
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    // 处理目标请求的方法
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Fibonacci::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request: %d", goal->order);

        if (goal->order < 0) {
            RCLCPP_ERROR(this->get_logger(), "%d is an invalid order, order must be non-negative! Rejecting...", goal->order);
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Received a goal: %d", goal->order);
        RCLCPP_DEBUG(this->get_logger(), "Goal ID: %s", rclcpp_action::to_string(uuid).c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理取消请求的方法
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 处理已接受目标的方法
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
        std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
    }

    // 执行目标的动作
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Excuting goal...");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto &sequence = feedback->partial_sequence;
        sequence.push_back(0);
        sequence.push_back(1);

        auto result = std::make_shared<Fibonacci::Result>();

        for (int i = 1; i < goal->order && rclcpp::ok(); i++) {
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                return;
            }

            sequence.push_back(sequence[i] + sequence[i - 1]);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publishing feedback...");
            std::this_thread::sleep_for(200ms);
        }

        if (rclcpp::ok()) {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
        }
    }
};

// 主函数，初始化ROS 2，创建并运行动作服务器，最后关闭ROS 2
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FibonacciActionServer>());
    rclcpp::shutdown();
    return 0;
}