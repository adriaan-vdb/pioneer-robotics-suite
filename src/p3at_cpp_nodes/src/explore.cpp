#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::placeholders;

class ExploreNode: public rclcpp::Node
{
public:
    ExploreNode(): Node("explore_node")
    {
        this->declare_parameter("range", 2.0);

        this->range_ = this->get_parameter("range").as_double();

         // min index and max index of the lidar data
        this->min_index = 270 + this->converter(-20.0);
        this->max_index = 270 + this->converter(20.0);

        this->nav_pub_ = this->create_publisher<Twist>("/nav_cmd_vel", 10);
        this->scan_sub_ = this->create_subscription<LaserScan>("scan", 10,
            std::bind(&ExploreNode::scan_callback, this, _1));

        this->turn_ = false;
        this->command_.linear.x = 0.0;
        this->command_.angular.z = 0.0;
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&ExploreNode::publish_cmd, this));
        RCLCPP_INFO(this->get_logger(), "Explore node has been started.");
        explore_drive();
    }

private:

    // Constantly checking lidar data
    void scan_callback(const LaserScan::SharedPtr msg){
        auto ranges = msg->ranges;
        for (int i = this->min_index; i < this->max_index; i++){
            if (verify_lidar_data(*msg, ranges[i])){
                if (ranges[i] < this->range_){
                    this->turn_ = true;
                    break;
                }
                else {
                    this->turn_ = false;
                }
            }
        }
        explore_drive();
    }

    // Lidar verification
    bool verify_lidar_data(LaserScan scan, float value){
        if (value < scan.range_min || value > scan.range_max){
            return false;
        }
        return true;
    }

    // Convert degrees to radians
    int converter(float degrees){
        double radian = degrees * M_PI / 180;
        return (int) (radian / 0.005817166529595852);
    }

    void publish_cmd(){
        this->nav_pub_->publish(this->command_);
    }

    void drive(double linear, double angular){
        this->command_.linear.x = linear;
        this->command_.angular.z = angular;
    }

    void turn(double rads){
        drive(0.0, 1.0);
        int duration = (int) (rads * 1000);
        this->publish_cmd();
        rclcpp::sleep_for(std::chrono::milliseconds(duration));
    }

    void explore_drive(){
        if (this->turn_){
            turn(1.57);
        }
        else {
            drive(0.5, 0.0);
        }
    }
    rclcpp::Publisher<Twist>::SharedPtr nav_pub_;
    rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int min_index, max_index;
    double range_;
    bool turn_;
    Twist command_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExploreNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}