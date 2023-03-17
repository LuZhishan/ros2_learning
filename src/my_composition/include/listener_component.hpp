#ifndef COMPOSITION__LISTENER_COMPONENT_HPP_
#define COMPOSITION__LISTENER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_composition
{

class Listener : public rclcpp::Node
{
public:
    Listener(const rclcpp::NodeOptions & options);

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace my_composition

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_
