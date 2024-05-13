#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "slam_toolbox/srv/save_map.hpp"
#include "std_msgs/msg/string.hpp"

using String = std_msgs::msg::String;
using Joy = sensor_msgs::msg::Joy;
using SaveMapServer = slam_toolbox::srv::SaveMap;
using namespace std::placeholders;

class JoyInputNode: public rclcpp::Node
{
public:
    JoyInputNode(): Node("joy_input_node")
    {
        this->joy_sub_ = this->create_subscription<Joy>("/joy", 10,
            std::bind(&JoyInputNode::joy_callback, this, _1));
        this->save_map_progress_ = false;
    }

private:

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

    void joy_callback(const Joy::SharedPtr msg){
        if (msg->buttons[2] == 1 && !this->save_map_progress_){
            RCLCPP_INFO(this->get_logger(), "Triangle pressed");
            threads_.push_back(std::thread(std::bind(&JoyInputNode::callSaveMapServer, this)));
        }
    }
    rclcpp::Subscription<Joy>::SharedPtr joy_sub_;

    std::vector<std::thread> threads_;
    
    bool save_map_progress_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyInputNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}