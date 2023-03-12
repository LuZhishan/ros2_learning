#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <chrono>
#include <memory>
#include <string>
#include <boost/optional.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

std::string target_frame_id, base_frame_id;

class TF_Publisher : public rclcpp::Node
{
public: TF_Publisher() : Node("TF_listener")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = this->create_wall_timer(1s, std::bind(&TF_Publisher::on_timer, this));
}
private:
    void on_timer()
    {
        try 
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer_->lookupTransform(
            target_frame_id,
            base_frame_id,
            rclcpp::Clock().now(),
            rclcpp::Duration::from_seconds(1));
            std::cout << "Find TF frome " << target_frame_id << " to " << base_frame_id << std::endl;
        }
        catch (tf2::TransformException & ex) 
        {
            std::cout << "Not find TF frome " << target_frame_id << " to " << base_frame_id << std::endl;
        }
    }
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
    if (argc != 3) 
    {
        std::cout << "you need to input target_frame_id and base_frame_id" << std::endl;
        std::cout << "Usage: ros2 run tf_publisher sub target_frame_id base_frame_id" << std::endl;
        return 1;
    }
    target_frame_id = argv[1]; base_frame_id = argv[2];

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TF_Publisher>());
    rclcpp::shutdown();
    return 0;
}
