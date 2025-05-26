#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node { // 继承 rclcpp::Node 类，表明这是一个 C++ ROS2 节点
  public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        // 创建 Subscription 对象，监听 topic 话题，队列大小为 10
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::callback, this, std::placeholders::_1));
    }

  private:
    void callback(const std_msgs::msg::String::SharedPtr message) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", message->data.c_str()); // 打印接收到的消息
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);                            // 初始化 ROS2
    rclcpp::spin(std::make_shared<MinimalSubscriber>()); // 运行节点
    rclcpp::shutdown();                                  // 关闭 ROS2
    return 0;
}