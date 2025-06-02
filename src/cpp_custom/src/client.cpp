#include "custom_interfaces/srv/coordinate_transformation.hpp"
#include "rclcpp/rclcpp.hpp"

#include <ctime>
#include <random>

using custom_interfaces::srv::CoordinateTransformation;
using namespace std::chrono_literals;

class TransformationClient : public rclcpp::Node {
  public:
    TransformationClient() : Node("transformation_client") {
        client_ = create_client<CoordinateTransformation>("transformation");
        timer_ = create_wall_timer(2s, std::bind(&TransformationClient::send_request, this));
    }

  private:
    rclcpp::Client<CoordinateTransformation>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_request() {
        auto request = std::make_shared<CoordinateTransformation::Request>();
        auto random_pair = generate_random();
        request->x = random_pair.first;
        request->y = random_pair.second;
        client_->async_send_request(request, std::bind(&TransformationClient::response_callback, this, std::placeholders::_1));
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

    void response_callback(const rclcpp::Client<CoordinateTransformation>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "Received response: radius=%f theta=%f", future.get()->radius, future.get()->theta);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformationClient>());
    rclcpp::shutdown();
    return 0;
}