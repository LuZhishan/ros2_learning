#include "tf_convert.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <memory>
#include <string>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

TfConvert::TfConvert():Node("TfConvert")
{

}

TfConvert::~TfConvert()
{
    
}

void TfConvert::convert_eigen_matrix_to_transform(const Eigen::Matrix4d & extrinsic_parameter,geometry_msgs::msg::TransformStamped & transform_stamped)
{
    //这里只用于表达语义,并没有实际作用,因为不会把这个tf发布到tf tree中去
    transform_stamped.header.frame_id = "velodyne_top";
    transform_stamped.child_frame_id = "camera";
    
    //平移
    transform_stamped.transform.translation.x= extrinsic_parameter(0,3);
    transform_stamped.transform.translation.y= extrinsic_parameter(1,3);
    transform_stamped.transform.translation.z= extrinsic_parameter(2,3);

    //旋转
    Eigen::Quaterniond quat(extrinsic_parameter.block<3, 3>(0, 0));
    quat.normalize();
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    transform_stamped.transform.rotation.w = quat.w();
}

void TfConvert::print_msg_tf(geometry_msgs::msg::TransformStamped msg_tf,std::string prefix)
{
    std::cout<<"*****************"<<prefix<<std::endl;

    std::cout<<"rotation"<<std::endl;
    std::cout<<msg_tf.transform.rotation.x<<std::endl;
    std::cout<<msg_tf.transform.rotation.y<<std::endl;
    std::cout<<msg_tf.transform.rotation.z<<std::endl;
    std::cout<<msg_tf.transform.rotation.w<<std::endl;

    std::cout<<"translation"<<std::endl;
    std::cout<<msg_tf.transform.translation.x<<std::endl;
    std::cout<<msg_tf.transform.translation.y<<std::endl;
    std::cout<<msg_tf.transform.translation.z<<std::endl;


    // double r,y,p;
    // double d1,d2,d3;
    tf2::Quaternion q(
        msg_tf.transform.rotation.x,
        msg_tf.transform.rotation.y,
        msg_tf.transform.rotation.z,
        msg_tf.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::cout<<"roll,pitch,yaw"<<std::endl;
    std::cout<<roll<<std::endl;
    std::cout<<pitch<<std::endl;
    std::cout<<yaw<<std::endl;
}


void TfConvert::example()
{
    Eigen::Matrix4d extrinsic_matrix;
    extrinsic_matrix<<0.06053494,-0.11761821,0.99121212,  0.3693644,
                    -0.99815968, -0.00357603,  0.06053489, -0.16847086,
                    -0.0035754,  -0.99305245, -0.11761823, -0.31519886,
                    0.,          0.,          0.,          1.;

    geometry_msgs::msg::TransformStamped  transform_stamped;
    convert_eigen_matrix_to_transform(extrinsic_matrix,transform_stamped);
    print_msg_tf(transform_stamped,"example:lidar to camera");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto tc = std::make_shared<TfConvert>();
    tc->example();
    rclcpp::spin(tc);
    rclcpp::shutdown();
    return 0;
}


