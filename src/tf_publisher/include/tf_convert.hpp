// #include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>

#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class TfConvert : public rclcpp::Node
{
public: 
    TfConvert();
    ~TfConvert();

    void example();
private:
    void convert_eigen_matrix_to_transform(const Eigen::Matrix4d & extrinsic_parameter,geometry_msgs::msg::TransformStamped & transform_stamped);
    void print_msg_tf(geometry_msgs::msg::TransformStamped msg_tf,std::string prefix); 
};