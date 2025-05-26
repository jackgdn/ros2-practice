#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals; // 启用时间自变量，例如 500ms

class MinimalPublisher : public rclcpp::Node { // 继承 rclcpp::Node 类，表明这是一个 C++ ROS2 节点
  public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); // 创建发布者，发布到 topic 话题，队列大小为 10
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::callback, this)); // 创建定时器，每 500ms 调用一次 callback 函数
    }

  private:
    void callback() {
        auto message = std_msgs::msg::String();                                    // 创建 std_msgs::msg::String 类型的消息
        message.data = "Hello, World: " + std::to_string(count_++);                // 设置消息内容
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // 打印发布信息
        publisher_->publish(message);                                              // 发布消息
    }

    rclcpp::TimerBase::SharedPtr timer_;                            // 定时器（智能指针管理）
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // 发布者（智能指针管理）
    size_t count_;                                                  // 计数器
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);                           // 初始化 ROS2
    rclcpp::spin(std::make_shared<MinimalPublisher>()); // 运行节点，直到节点退出
    rclcpp::shutdown();                                 // 关闭 ROS2
    return 0;
}