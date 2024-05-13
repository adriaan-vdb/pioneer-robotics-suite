#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "sensor_msgs/msg/joy.hpp"

using Joy = sensor_msgs::msg::Joy;
using Snapshot = rosbag2_interfaces::srv::Snapshot;
using LaserScan = sensor_msgs::msg::LaserScan;

class EStopNode: public rclcpp::Node
{
public:
    EStopNode(): Node("emergency_node")
    {
        this->declare_parameter("range_min_check", -30.0);
        this->declare_parameter("range_max_check", 30.0);
        this->declare_parameter("keepout_range", 1.0);

        this->range = this->get_parameter("keepout_range").as_double();
        double range_min_check = this->get_parameter("range_min_check").as_double();
        double range_max_check = this->get_parameter("range_max_check").as_double();

        this->min_index = 240 + this->converter(range_min_check);
        this->max_index = 240 + this->converter(range_max_check);

        this->min_range = -1.3962600231170654;
        this->lidar_sub_ = this->create_subscription<LaserScan>(
            "/scan", 10, std::bind(&EStopNode::lidar_callback, this, std::placeholders::_1));

        this->stop_ = false;
        this->joy_sub_ = this->create_subscription<Joy>("/joy", 10,
            std::bind(&EStopNode::joy_callback, this, std::placeholders::_1));
    }

private:

    void joy_callback(const Joy::SharedPtr msg){
        if (msg->buttons[3] == 1 && this->stop_){
            RCLCPP_INFO(this->get_logger(), "Emergency Stop Released");
            this->stop_ = false;
        }
    }

    void callSnapshot(){
        this->stop_ = true;
        auto client = this->create_client<Snapshot>("/rosbag2_recorder/snapshot");
        while (!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), "Waiting for the snapshot server to be up...");
        }

        auto request = std::make_shared<Snapshot::Request>();
        auto future = client->async_send_request(request);
        try {
            auto response = future.get();
            if (response->success){
                RCLCPP_INFO(this->get_logger(), "Snapshot taken");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Error while taking snapshot");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error while calling the service");
        }
    }

    bool verify_lidar_data(LaserScan scan, float value){
        if (value < scan.range_min || value > scan.range_max){
            return false;
        }
        return true;
    }

    int converter(float degrees){
        double radian = degrees * M_PI / 180;
        return (int) (radian / 0.004370140843093395);
    }

    void lidar_callback(const LaserScan::SharedPtr msg){
        auto ranges = msg->ranges;
        for (int i = this->min_index; i < this->max_index; i++){
            if (verify_lidar_data(*msg, ranges[i])){
                if (ranges[i] < this->range){
                    if (!this->stop_){
                        this->stop_ = true;
                        RCLCPP_INFO(this->get_logger(), "Taking Snapshot");
                        threads_.push_back(std::thread(std::bind(&EStopNode::callSnapshot, this)));
                        RCLCPP_INFO(this->get_logger(), "Emergency Stop");
                    }
                }
            }
        }
    }

    // double range_min_check, range_max_check;
    int min_index, max_index;
    double min_range, range;

    std::vector<std::thread> threads_;

    bool stop_;
    rclcpp::Subscription<LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EStopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}