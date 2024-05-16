#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "sensor_msgs/msg/imu.hpp"

using TransBroadcaster = tf2_ros::TransformBroadcaster;
using Odometry = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Quaternion = tf2::Quaternion;
using Imu = sensor_msgs::msg::Imu;

class OdomNode: public rclcpp::Node
{
public:
    OdomNode(): Node("odom_node"),
        theta_(0.0), x_(0.0), y_(0.0)
    {
        this->twist_sub_ = this->create_subscription<Twist>("/cmd_vel", 10,
            std::bind(&OdomNode::twist_callback, this, std::placeholders::_1));
        this->odom_pub_ = this->create_publisher<Odometry>("/odom", 10);
        this->imu_sub_ = this->create_subscription<Imu>("/imu/data_raw", 10,
            std::bind(&OdomNode::imu_callback, this, std::placeholders::_1));
        
        this->tf_broadcaster_ = std::make_unique<TransBroadcaster>(*this);
        this->tf.header.frame_id = "odom";
        this->tf.child_frame_id = "base_link";
        this->prev_time_ = this->get_clock()->now();
        this->tf.header.stamp = this->prev_time_;
        this->tf.transform.translation.x = 0.0;
        this->tf.transform.translation.y = 0.0;
        this->tf.transform.translation.z = 0.0;
        this->tf.transform.rotation.x = 0.0;
        this->tf.transform.rotation.y = 0.0;
        this->tf.transform.rotation.z = 0.0;
        this->tf.transform.rotation.w = 1.0;

        this->tf_broadcaster_->sendTransform(this->tf);
        RCLCPP_INFO(this->get_logger(), "Odometry Node has been started");
    }

private:

    void imu_callback(const Imu::SharedPtr msg)
    {
        this->current_imu_ = *msg;
    }

    void twist_callback(const Twist::SharedPtr msg)
    {
        rclcpp::Time msg_time = this->get_clock()->now();
        rclcpp::Duration dt = msg_time - this->prev_time_;

        double linear = msg->linear.x;
        double angular = msg->angular.z;

        double d_s = linear * dt.seconds();
        double roll, pitch, yaw;
        Quaternion p(
            this->current_imu_.orientation.x,
            this->current_imu_.orientation.y,
            this->current_imu_.orientation.z,
            this->current_imu_.orientation.w);
        tf2::Matrix3x3 m(p);
        m.getRPY(roll, pitch, yaw);
        this->theta_ = yaw;

        this->x_ += d_s * cos(this->theta_);
        this->y_ += d_s * sin(this->theta_);

        Quaternion q;
        q.setRPY(0.0, 0.0, this->theta_);

        this->tf.header.stamp = msg_time;
        this->tf.transform.translation.x = this->x_;
        this->tf.transform.translation.y = this->y_;
        this->tf.transform.rotation.x = q.x();
        this->tf.transform.rotation.y = q.y();
        this->tf.transform.rotation.z = q.z();
        this->tf.transform.rotation.w = q.w();

        this->prev_time_ = msg_time;

        this->tf_broadcaster_->sendTransform(this->tf);
    }

    rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    TransformStamped tf;
    rclcpp::Time prev_time_;
    double theta_, x_, y_;

    Imu current_imu_;
    std::unique_ptr<TransBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}