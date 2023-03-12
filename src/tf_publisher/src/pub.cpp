#include "pub.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

TF_Publisher::TF_Publisher():Node("TF_publisher")
{
    this->declare_parameter<std::string>("father_frame_id", "base_link");
    this->declare_parameter<std::string>("child_frame_id", "velodyne_top");
    this->get_parameter("father_frame_id", father_frame_id_);
    this->get_parameter("child_frame_id", child_frame_id_);

    this->declare_parameter<bool>("double_frame", false);
    this->get_parameter("double_frame", double_frame_);

    this->declare_parameter<std::string>("father_frame2_id", "map");
    this->declare_parameter<std::string>("child_frame2_id", "base_link");
    this->get_parameter("father_frame2_id", father_frame2_id_);
    this->get_parameter("child_frame2_id", child_frame2_id_);

    this->declare_parameter<std::string>("father_frame3_id", "");
    this->declare_parameter<std::string>("child_frame3_id", "");
    this->get_parameter("father_frame3_id", father_frame3_id_);
    this->get_parameter("child_frame3_id", child_frame3_id_);

    this->declare_parameter<bool>("use_topic_time", false);
    this->get_parameter("use_topic_time", use_topic_time_);
    this->declare_parameter<std::string>("topic_name", "/sensing/lidar/top/rectified/pointcloud");
    this->get_parameter("topic_name", topic_name);


    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if(use_topic_time_)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, rclcpp::SensorDataQoS().keep_last(1), std::bind(&TF_Publisher::topic_callback, this, _1));
    }
    else
    {
        timer_ = this->create_wall_timer(100ms, std::bind(&TF_Publisher::on_timer, this));
    }
}

void TF_Publisher::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    rclcpp::Time time_stamp_ = msg->header.stamp;
    send_tf(time_stamp_, father_frame_id_,child_frame_id_,0.6,0,2,0,0,0);

    if(double_frame_)
    {
        send_tf(time_stamp_, father_frame2_id_,child_frame2_id_,0,0,1,0,0,0);
    }

    //velodyne_top to front_camera
    double roll = -1.74095311374;
    double pitch = 0.00673870311114;
    double yaw = -1.55412676164;
    double x = 0.344678021799;
    double y = -0.183680316566;
    double z = -0.334097918002;
    send_tf(time_stamp_, father_frame3_id_,child_frame3_id_,x,y,z,roll,pitch,yaw);
}

void TF_Publisher::on_timer()
{    
    rclcpp::Time time_stamp_ = this->get_clock()->now();
    send_tf(time_stamp_, father_frame_id_,child_frame_id_,0.6,0,2,0,0,0);

    if(double_frame_)
    {
        send_tf(time_stamp_, father_frame2_id_,child_frame2_id_,0,0,1,0,0,0);
    }

    //velodyne_top to front_camera
    double roll = -1.74095311374;
    double pitch = 0.00673870311114;
    double yaw = -1.55412676164;
    double x = 0.344678021799;
    double y = -0.183680316566;
    double z = -0.334097918002;
    send_tf(time_stamp_, father_frame3_id_,child_frame3_id_,x,y,z,roll,pitch,yaw);
}


void TF_Publisher::send_tf(rclcpp::Time time_stamp, std::string father_frame_id,std::string child_frame_id,double x,double y,double z,double roll,double pitch,double yaw)
{
    geometry_msgs::msg::TransformStamped frame_published;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    frame_published.header.stamp = time_stamp;
    frame_published.header.frame_id = father_frame_id;      // 父节点
    frame_published.child_frame_id = child_frame_id;        // 子节点
    frame_published.transform.translation.x = x;
    frame_published.transform.translation.y = y;
    frame_published.transform.translation.z = z;
    
    frame_published.transform.rotation.x = q.x();
    frame_published.transform.rotation.y = q.y();
    frame_published.transform.rotation.z = q.z();
    frame_published.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(frame_published);
    std::cout << "Publisher TF frome " << father_frame_id << " to " << child_frame_id << std::endl;
}


int main(int argc, char * argv[])
{
    // setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TF_Publisher>());
    rclcpp::shutdown();
    return 0;
}
