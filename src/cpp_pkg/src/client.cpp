#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalClient : public rclcpp::Node {
  public:
    MinimalClient() : Node("mininal_client") {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("service");
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalClient::send_request, this));
    }

  private:
    void timer_callback(const rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "Response: %ld", future.get()->sum);
    }

    void send_request() {
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted.");
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "Service not available");
            }
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = i_++;
        request->b = i_++;
        client_->async_send_request(request, std::bind(&MinimalClient::timer_callback, this, std::placeholders::_1));
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t i_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalClient>());
    rclcpp::shutdown();
    return 0;
}