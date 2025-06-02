#include "rclcpp/rclcpp.hpp"

using namespace std;

class MinimalParameterNode : public rclcpp::Node {
  public:
    MinimalParameterNode() : Node("minimal_parameter_node") {
        this->declare_parameter("custom_parameter", "Hello, World!");
        timer_ = this->create_wall_timer(1000ms, bind(&MinimalParameterNode::callback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;

    void callback() {
        string param = this->get_parameter("custom_parameter").as_string();
        RCLCPP_INFO(this->get_logger(), "Custom parameter: %s", param.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalParameterNode>());
    rclcpp::shutdown();
    return 0;
}