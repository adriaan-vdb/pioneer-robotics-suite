#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::placeholders;

class TestScan: public rclcpp::Node
{
public:
    TestScan(): Node("explore_node")
    {
        this->scan_sub_ = this->create_subscription<LaserScan>("/scan", 10,
            std::bind(&TestScan::scan_callback, this, _1));
        this->increment_ = 0.00581718236207962;
        this->angle_min_ = -1.570874810218811;
    }

private:

    void scan_callback(const LaserScan::SharedPtr msg)
    {
        for (int i = 0; i < msg->ranges.size(); i = i + 10){
            double angle = i * this->increment_ + this->angle_min_;
            int angle_deg = converter(angle);
            RCLCPP_INFO(this->get_logger(), "Angle: %d, Range: %f", angle_deg, msg->ranges[i]);
        }
    }

    int converter(double angle){
        return angle * 180 / M_PI;
    }

    rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
    double increment_, angle_min_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}