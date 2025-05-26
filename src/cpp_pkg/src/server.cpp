#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MinimalServer : public rclcpp::Node {
  public:
    MinimalServer() : Node("add_two_ints_server") {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>("service",
                                                                             std::bind(&MinimalServer::handle, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Server ready.");
    }

  private:
    void handle(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                const example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Request: a=%ld, b=%ld", request->a, request->b);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalServer>());
    rclcpp::shutdown();
    return 0;
}