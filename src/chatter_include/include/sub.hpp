#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
    Subscriber() : Node("subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&Subscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String &msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};