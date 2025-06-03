#include "rclcpp/rclcpp.hpp"

using namespace std;

class MinimalParameterNode : public rclcpp::Node {
  public:
    // 构造函数，初始化节点并声明参数
    MinimalParameterNode() : Node("minimal_parameter_node") {
        this->declare_parameter("custom_parameter", "Hello, World!");
        timer_ = this->create_wall_timer(1000ms, bind(&MinimalParameterNode::callback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;

    // 定时器回调函数，读取并打印参数值
    void callback() {
        string param = this->get_parameter("custom_parameter").as_string();
        RCLCPP_INFO(this->get_logger(), "Custom parameter: %s", param.c_str());
    }
};

int main(int argc, char **argv) {
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建并运行节点
    rclcpp::spin(std::make_shared<MinimalParameterNode>());
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}