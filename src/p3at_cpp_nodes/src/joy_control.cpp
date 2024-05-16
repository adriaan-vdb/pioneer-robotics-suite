#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "slam_toolbox/srv/save_map.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

using Trigger = std_srvs::srv::Trigger;
using String = std_msgs::msg::String;
using SaveMapServer = slam_toolbox::srv::SaveMap;
using Twist = geometry_msgs::msg::Twist;
using Joy = sensor_msgs::msg::Joy;
using Snapshot = rosbag2_interfaces::srv::Snapshot;
using LaserScan = sensor_msgs::msg::LaserScan;
using Float32 = std_msgs::msg::Float32;
using namespace std::placeholders;

class JoyControlNode: public rclcpp::Node
{
public:
    JoyControlNode(): Node("joy_control_node")
    {
        // Declare parameter:
        this->declare_parameter("range_min_check", -30.0);
        this->declare_parameter("range_max_check", 30.0);
        this->declare_parameter("keepout_range", 1.0);

        this->range = this->get_parameter("keepout_range").as_double();
        double range_min_check = this->get_parameter("range_min_check").as_double();
        double range_max_check = this->get_parameter("range_max_check").as_double();

        // min index and max index of the lidar data
        this->min_index = 240 + this->converter(range_min_check);
        this->max_index = 240 + this->converter(range_max_check);

        // Indicate callback service
        this->callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        this->toggle_stop_service_ = this->create_service<Trigger>(
            "/p3at/toggle_stop",
            std::bind(&JoyControlNode::toggle_stop_service, this, _1, _2),
            rmw_qos_profile_services_default,
            this->callback_group_);

        // min range of the lidar dataS
        this->min_range = -1.3962600231170654;
        this->stop_ = false;
        this->safety_off_ = false;
        this->manual_ = true;

        // Subscribe to joystick and command velocity.
        this->lidar_sub_ = this->create_subscription<LaserScan>(
            "/scan", 10, std::bind(&JoyControlNode::lidar_callback, this, std::placeholders::_1));
        this->joy_sub_ = this->create_subscription<Joy>("/joy", 1,
            std::bind(&JoyControlNode::joy_callback, this, std::placeholders::_1));
        this->joy_twist_sub_ = this->create_subscription<Twist>("/joy_cmd_vel", 10,
            std::bind(&JoyControlNode::joy_velo_callback, this, std::placeholders::_1));
        this->nav_twist_sub_ = this->create_subscription<Twist>("/nav_cmd_vel", 10,
            std::bind(&JoyControlNode::nav_velo_callback, this, std::placeholders::_1));

        // Initiate Publishers:
        // cmd_vel, gear, speed, and steering
        this->twist_pub_ = this->create_publisher<Twist>("/cmd_vel", 10);
        this->gear_pub_ = this->create_publisher<String>("gear", 10);
        this->safety_ = this->create_publisher<String>("safety", 10);
        this->speed_pub_ = this->create_publisher<Float32>("speed", 10);
        RCLCPP_INFO(this->get_logger(), "Joy/Velocity Node has been started");

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
            std::bind(&JoyControlNode::publish_cmd_vel, this));
    }

private:

    void toggle_stop_service(const Trigger::Request::SharedPtr request,
                            const Trigger::Response::SharedPtr response){
        this->stop_ = !this->stop_;
        response->success = true;
        response->message = this->stop_ ? "Emergency Stop" : "Emergency Stop Released";
    }

    // Call the save map service
    void callSaveMapServer(){
        save_map_progress_ = true;
        auto client = this->create_client<SaveMapServer>("/slam_toolbox/save_map");
        while (!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        String name;
        name.data = "map";
        auto request = std::make_shared<SaveMapServer::Request>();
        request->name = name;

        auto future = client->async_send_request(request);
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d", response->result);
            save_map_progress_ = false;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error while calling the service");
        }
    }

    // Record the velocity from the joystick if manual mode
    void joy_velo_callback(const Twist::SharedPtr msg){
        if (!this->stop_ && this->manual_){
            this->current_twist_ = *msg;
            Float32 speed;
            speed.data = msg->linear.x;
            this->speed_pub_->publish(speed);
        }
    }

    // Record the velocity from the navigation if navigation mode
    void nav_velo_callback(const Twist::SharedPtr msg){
        if (!this->stop_ && !this->manual_){
            this->current_twist_ = *msg;
            Float32 speed;
            speed.data = msg->linear.x;
            this->speed_pub_->publish(speed);
        }
    }

    // Record the joystick button and corresponding actions
    void joy_callback(const Joy::SharedPtr msg){
        if (msg->buttons[3] == 1 && this->stop_){
            RCLCPP_INFO(this->get_logger(), "Emergency Stop Released");
            this->stop_ = false;
        } else if (msg->axes[7] == 1.0 && this->safety_off_){
            this->safety_off_ = false;
            RCLCPP_WARN(this->get_logger(), "Safety %d", this->safety_off_ );
            String text;
            text.data = "ON";
            this->safety_->publish(text);
        } else if (msg->axes[7] == -1.0 && !this->safety_off_){
            this->safety_off_ = true;
            RCLCPP_WARN(this->get_logger(), "Safety %d", this->safety_off_ );
            String text;
            text.data = "OFF";
            this->safety_->publish(text);
        } else if (msg->buttons[0] == 1 && !this->manual_){
            this->manual_ = true;
            RCLCPP_INFO(this->get_logger(), "Manual Mode");
            String text;
            text.data = "Manual";
            this->gear_pub_->publish(text);
        } else if (msg->buttons[1] == 1 && this->manual_){
            this->manual_ = false;
            RCLCPP_INFO(this->get_logger(), "Navigation Mode");
            String text;
            text.data = "Navigation";
            this->gear_pub_->publish(text);
        } else if (msg->buttons[2] == 1 && !this->save_map_progress_){
            RCLCPP_INFO(this->get_logger(), "Saving Map");
            threads_.push_back(std::thread(std::bind(&JoyControlNode::callSaveMapServer, this)));
        }
    }

    // Call the snapshot service
    void callSnapshot(){
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
        return (int) (radian / 0.004370140843093395);
    }

    // Constantly publish the current twist mesaage (40 Hz)
    void publish_cmd_vel(){
        if (this->stop_){
            this->emergency_stop();
        }
        // RCLCPP_INFO(this->get_logger(), "%d", this->stop_ );
        this->twist_pub_->publish(this->current_twist_);
    }

    // Apply emergency stop which publish twist 0 0 0
    void emergency_stop(){
        Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        this->current_twist_ = twist;
        Float32 speed;
        speed.data = 0.0;
        this->speed_pub_->publish(speed);
    }

    // Constantly checking lidar data
    void lidar_callback(const LaserScan::SharedPtr msg){
        auto ranges = msg->ranges;
        for (int i = this->min_index; i < this->max_index; i++){
            if (verify_lidar_data(*msg, ranges[i])){
                if (ranges[i] < this->range){
                    if (!this->stop_ && !this->safety_off_){
                        this->stop_ = true;
                        this->emergency_stop();
                        RCLCPP_INFO(this->get_logger(), "Taking Snapshot");
                        threads_.push_back(std::thread(std::bind(&JoyControlNode::callSnapshot, this)));
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

    bool stop_, safety_off_, manual_;
    bool save_map_progress_;

    // Keep record of the latest twist message
    Twist current_twist_;
    rclcpp::Subscription<LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<Twist>::SharedPtr joy_twist_sub_, nav_twist_sub_;

    rclcpp::Publisher<Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<String>::SharedPtr gear_pub_, safety_;
    rclcpp::Publisher<Float32>::SharedPtr speed_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<Trigger>::SharedPtr toggle_stop_service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}