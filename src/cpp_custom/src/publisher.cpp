#include "custom_interfaces/msg/coordinate.hpp"
#include "rclcpp/rclcpp.hpp"

#include <ctime>
#include <random>

using namespace std::chrono_literals;
using custom_interfaces::msg::Coordinate;

class CoordinatePublisher : public rclcpp::Node {
  public:
    CoordinatePublisher() : Node("coordinate_publisher") {
        publisher_ = this->create_publisher<Coordinate>("coordinate", 10);
        timer_ = this->create_wall_timer(2s, std::bind(&CoordinatePublisher::callback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Coordinate>::SharedPtr publisher_;

    void callback() {
        auto message = Coordinate();
        auto random_pair = generate_random();
        message.x = random_pair.first;
        message.y = random_pair.second;
        RCLCPP_INFO(this->get_logger(), "Publishing: (%f, %f)", message.x, message.y);
        publisher_->publish(message);
    }

    std::pair<double, double> generate_random() {
        std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(-180.0, 180.0);
        generator.seed(time(0));
        double x = distribution(generator);
        generator.seed(time(0) + 1);
        double y = distribution(generator);
        return std::make_pair(x, y);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinatePublisher>());
    rclcpp::shutdown();
    return 0;
}