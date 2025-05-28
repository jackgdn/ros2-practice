#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MinimalServer : public rclcpp::Node {
  public:
    // 构造函数，初始化节点并创建服务
    MinimalServer() : Node("add_two_ints_server") {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>("service",
                                                                             std::bind(&MinimalServer::handle, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Server ready.");
    }

  private:
    // 处理服务请求的回调函数
    void handle(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                const example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Request: a=%ld, b=%ld", request->a, request->b);
    }

    // 服务的共享指针
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

// 主函数，初始化 ROS 2，创建并自旋服务器节点，最后关闭
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalServer>());
    rclcpp::shutdown();
    return 0;
}