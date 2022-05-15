#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>

#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class TF_Publisher : public rclcpp::Node
{
public: 
    TF_Publisher();
private:
    void on_timer();
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void send_tf(rclcpp::Time time_stamp, 
                std::string father_frame_id,std::string child_frame_id,
                double x,double y,double z,double roll,double pitch,double yaw);
    
    std::string father_frame_id_, child_frame_id_;
    bool double_frame_;
    std::string father_frame2_id_, child_frame2_id_;

    std::string father_frame3_id_, child_frame3_id_;
    std::string father_frame4_id_, child_frame4_id_;
    std::string father_frame5_id_, child_frame5_id_;

    bool use_topic_time_;
    std::string topic_name;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};