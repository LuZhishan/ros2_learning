#ifndef COMPOSITION__TALKER_COMPONENT_HPP_
#define COMPOSITION__TALKER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_composition
{

class Talker : public rclcpp::Node
{
public:
    Talker(const rclcpp::NodeOptions & options);

protected:
    void on_timer();

private:
    size_t count_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace my_composition

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_
