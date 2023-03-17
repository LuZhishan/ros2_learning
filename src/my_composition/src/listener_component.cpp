#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "listener_component.hpp"

namespace my_composition
{

Listener::Listener(const rclcpp::NodeOptions & options)
: Node("listener", options)
{
    auto callback = [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
    {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        std::flush(std::cout);
    };
    sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
}

}  // namespace my_composition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_composition::Listener)