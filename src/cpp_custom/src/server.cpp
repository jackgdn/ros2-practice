#include "custom_interfaces/srv/coordinate_transformation.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>

using custom_interfaces::srv::CoordinateTransformation;
using std::placeholders::_1;
using std::placeholders::_2;

class TransformationServer : public rclcpp::Node {
  public:
    TransformationServer() : Node("transformation_server") {
        service_ =
            create_service<CoordinateTransformation>("transformation", std::bind(&TransformationServer::handle, this, _1, _2));
    }

  private:
    rclcpp::Service<CoordinateTransformation>::SharedPtr service_;

    void handle(const CoordinateTransformation::Request::SharedPtr request,
                const CoordinateTransformation::Response::SharedPtr response) {
        double x = request->x, y = request->y;
        RCLCPP_INFO(this->get_logger(), "Received request: x=%f y=%f", x, y);
        response->radius = std::sqrt(x * x + y * y);
        response->theta = std::atan2(y, x);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformationServer>());
    rclcpp::shutdown();
    return 0;
}