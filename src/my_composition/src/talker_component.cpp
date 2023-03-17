#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "talker_component.hpp"

using namespace std::chrono_literals;

namespace my_composition
{
// 创建一个Talker“组件”，将通用rclcpp:：Node基类子类化。
// 组件被构建到共享库中，因此不编写自己的主要函数。
// 使用组件共享库的进程将把类实例化为ROS节点。
Talker::Talker(const rclcpp::NodeOptions & options)
: Node("talker", options), count_(0)
{
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(1s, std::bind(&Talker::on_timer, this));
}

void Talker::on_timer()
{
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Hello World: " + std::to_string(++count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
    std::flush(std::cout);
    pub_->publish(std::move(msg));
}

}  // namespace my_composition

#include "rclcpp_components/register_node_macro.hpp"

// 使用class_loader注册组件。
// 这是一种入口点，允许在将组件库加载到正在运行的进程中时发现组件。
RCLCPP_COMPONENTS_REGISTER_NODE(my_composition::Talker)
