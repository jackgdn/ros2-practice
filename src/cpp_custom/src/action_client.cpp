#include "custom_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using custom_interfaces::action::Fibonacci;
using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

// 斐波那契动作客户端类，继承自 rclcpp::Node
class FibonacciActionClient : public rclcpp::Node {
  public:
    // 构造函数，初始化节点并创建动作客户端和定时器
    FibonacciActionClient() : Node("fibonacci_action_client") {
        this->action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
        timer_ = this->create_wall_timer(200ms, std::bind(&FibonacciActionClient::send_goal, this));
    }

    // 检查目标是否完成
    bool is_goal_done() { return goal_done_; }

    // 发送动作目标
    void send_goal() {
        this->timer_->cancel();

        if (!this->action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server unavailable.");
            this->goal_done_ = true;
            return;
        }

        auto goal = Fibonacci::Goal();
        goal.order = this->random_order();
        RCLCPP_INFO(this->get_logger(), "Sending goal with order %d", goal.order);

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&FibonacciActionClient::result_callback, this, _1);

        this->future_ = this->action_client_->async_send_goal(goal, send_goal_options);
    }

    // 生成一个随机的斐波那契序列长度
    int random_order() { return time(nullptr) % 20 - 9; }

  private:
    rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;       // 动作客户端指针
    rclcpp::TimerBase::SharedPtr timer_;                              // 定时器指针
    std::shared_future<std::shared_ptr<FibonacciGoalHandle>> future_; // 发送目标的未来结果
    bool goal_done_ = false;                                          // 标志目标是否完成

    // 处理目标响应的回调函数
    void goal_response_callback(std::shared_future<FibonacciGoalHandle::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected.");
            this->goal_done_ = true;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal accepted.");
    }

    // 处理目标反馈的回调函数
    void feedback_callback(FibonacciGoalHandle::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback) {
        std::stringstream ss;
        for (auto number : feedback->partial_sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "Received feedback: %s", ss.str().c_str());
    }

    // 处理目标结果的回调函数
    void result_callback(const FibonacciGoalHandle::WrappedResult &result) {
        this->goal_done_ = true;

        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal aborted.");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal canceled.");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code,");
            return;
        }

        std::stringstream ss;
        for (auto number : result.result->sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "Received result: %s", ss.str().c_str());
    }
};

// 主函数，初始化 ROS 2，创建动作客户端实例并发送目标，等待目标完成
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<FibonacciActionClient>();
    action_client->send_goal();
    while (!action_client->is_goal_done()) {
        rclcpp::spin_some(action_client);
    }
    rclcpp::shutdown();
    return 0;
}