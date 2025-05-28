#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 定义一个继承自 rclcpp::Node 的类，用于实现一个最小化的客户端
class MinimalClient : public rclcpp::Node {
  public:
    // 构造函数，初始化节点并创建客户端和服务定时器
    MinimalClient() : Node("mininal_client") {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("service");
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalClient::send_request, this));
    }

  private:
    // 定时器回调函数，处理服务响应
    void timer_callback(const rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "Response: %ld", future.get()->sum);
    }

    // 发送请求到服务端
    void send_request() {
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted.");
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "Service not available.");
            }
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = i_++;
        request->b = i_++;
        client_->async_send_request(request, std::bind(&MinimalClient::timer_callback, this, std::placeholders::_1));
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_; // 服务客户端指针
    rclcpp::TimerBase::SharedPtr timer_;                                    // 定时器指针
    size_t i_ = 0;                                                          // 用于生成请求数据的计数器
};

// 主函数，初始化 ROS 2 环境，创建并运行客户端节点，最后关闭 ROS 2 环境
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalClient>());
    rclcpp::shutdown();
    return 0;
}