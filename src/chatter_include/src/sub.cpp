#include "sub.hpp"

void Subscriber::topic_callback(const std_msgs::msg::String &msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}