#include "custom_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using custom_interfaces::action::Fibonacci;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class FibonacciActionServer : public rclcpp::Node {
  public:
    FibonacciActionServer() : Node("fibonacci_action_server") {
        this->action_server_ =
            rclcpp_action::create_server<Fibonacci>(this,
                                                    "fibonacci",
                                                    std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
                                                    std::bind(&FibonacciActionServer::handle_cancel, this, _1),
                                                    std::bind(&FibonacciActionServer::handle_accepted, this, _1));
    }

  private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

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

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
        std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
    }

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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FibonacciActionServer>());
    rclcpp::shutdown();
    return 0;
}