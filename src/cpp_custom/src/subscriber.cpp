#include "custom_interfaces/msg/coordinate.hpp"
#include "rclcpp/rclcpp.hpp"

using custom_interfaces::msg::Coordinate;

class CoordinateSubscriber : public rclcpp::Node {
  public:
    CoordinateSubscriber() : Node("coordinate_subscriber") {
        subscriber_ = this->create_subscription<Coordinate>(
            "coordinate", 10, std::bind(&CoordinateSubscriber::callback, this, std::placeholders::_1));
    }

  private:
    rclcpp::Subscription<Coordinate>::SharedPtr subscriber_;

    void callback(const Coordinate::SharedPtr message) {
        RCLCPP_INFO(this->get_logger(), "Received: (%f, %f)", message->x, message->y);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateSubscriber>());
    rclcpp::shutdown();
    return 0;
}