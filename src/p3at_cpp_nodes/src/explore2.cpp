#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

using namespace std::placeholders;

class ExploreNode : public rclcpp::Node
{
public:
    ExploreNode() : Node("explore_node")
    {
        this->declare_parameter("range", 2.0);

        this->range_ = this->get_parameter("range").as_double();

        this->count = 0;
        this->start_time_ = this->get_clock()->now();

        // min index and max index of the lidar data
        this->min_index = 270 + this->converter(-40.0);
        this->max_index = 270 + this->converter(40.0);

        this->nav_pub_ = this->create_publisher<Twist>("/nav_cmd_vel", 10);
        this->scan_sub_ = this->create_subscription<LaserScan>("/scan", 10,
                                                               std::bind(&ExploreNode::scan_callback, this, _1));
        
        this->pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
            "/pose", 10, std::bind(&ExploreNode::pose_callback, this, _1));

        this->turn_ = false;
        this->command_.linear.x = 0.0;
        this->command_.angular.z = 0.0;
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                               std::bind(&ExploreNode::publish_cmd, this));

        this->timer_repeat_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                      std::bind(&ExploreNode::explore_drive, this));
        RCLCPP_INFO(this->get_logger(), "Explore node has been started.");
    }

private:
    // Constantly checking lidar data
    void scan_callback(const LaserScan::SharedPtr msg)
    {
        auto ranges = msg->ranges;
        for (int i = this->min_index; i < this->max_index; i++)
        {
            if (verify_lidar_data(*msg, ranges[i]) && outside_boundary())
            {
                if (ranges[i] < this->range_ && ranges[i] > 0.1)
                {
                    this->turn_ = true;
                    break;
                }
                else
                {
                    this->turn_ = false;
                }
            }
        }
    }

    // Lidar verification
    bool verify_lidar_data(const LaserScan &scan, float value)
    {
        if (value < scan.range_min || value > scan.range_max)
        {
            return false;
        }
        return true;
    }

    // Pose callback to update current pose
    void pose_callback(const PoseWithCovarianceStamped::SharedPtr msg)
    {
        this->current_pose_ = msg->pose.pose;
    }

    // Drive until an obstacle is seen, or (x, y) pose +- 15 meters from origin starting point
    bool outside_boundary()
    {
        double x = this->current_pose_.position.x;
        double y = this->current_pose_.position.y;
        if (abs(x) < 10.0 && abs(y) < 10.0)
        {
            return false; // inside boundary
        }
        return true; // outside boundary, turn
    }

    // Convert degrees to radians
    int converter(float degrees)
    {
        double radian = degrees * M_PI / 180;
        return (int)(radian / 0.005817166529595852);
    }

    void publish_cmd()
    {
        this->nav_pub_->publish(this->command_);
    }

    void drive(double linear, double angular)
    {
        this->command_.linear.x = linear;
        this->command_.angular.z = angular;
    }

    void turn(double rads)
    {
        drive(0.0, 1.0);
        int duration = (int)(rads * 1000);
        this->turn_time_ = this->get_clock()->now();
        rclcpp::Duration dt = this->get_clock()->now() - this->turn_time_;
        while (dt.seconds() < duration)
        {
            this->publish_cmd();
            dt = this->get_clock()->now() - this->turn_time_;
        }
    }

    void explore_drive()
    {
        rclcpp::Duration duration_ = this->get_clock()->now() - this->start_time_;
        RCLCPP_INFO(this->get_logger(), "Turn %d, and %.2f elapsed.", this->count, duration_.seconds() / 60.0);
        if (this->turn_)
        {
            drive(0.0, 1.0);
            count++;
        }
        else
        {
            drive(0.5, 0.0);
        }
    }

    rclcpp::Publisher<Twist>::SharedPtr nav_pub_;
    rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_repeat_;

    int min_index, max_index;
    double range_;
    bool turn_;
    Twist command_;
    rclcpp::Time start_time_, turn_time_;
    int count;
    geometry_msgs::msg::Pose current_pose_; // Current pose of the robot
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExploreNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
