#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "p3at_interface/msg/target_goal.hpp"
#include "p3at_interface/msg/object_info.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>

using Int32 = std_msgs::msg::Int32;
using String = std_msgs::msg::String;
using ObjectInfo = p3at_interface::msg::ObjectInfo;
using TargetGoal = p3at_interface::msg::TargetGoal;
using Pose = geometry_msgs::msg::Pose;
using PoseWithCStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::placeholders;

const double SPEED = 0.8;

// class NodeList{
// public:
//     double x;
//     double y;
//     std::string description;
//     NodeList* next;

//     NodeList(double x, double y, std::string description){
//         this->x = x;
//         this->y = y;
//         this->description = description;
//         next = nullptr;
//     }
// };

// class LinkedList {
// NodeList* ind;
// NodeList* head;

// public:
//     LinkedList(){
//         head = nullptr;
//         ind = nullptr;
//     }

//     void insertNode(double x, double y, std::string description){
//         NodeList new_node = NodeList(x, y, description);
//         if (head == nullptr){
//             head = &new_node;
//         } else {
//             NodeList* ptr = head;
//             while (ptr->next != nullptr){
//                 ptr = ptr->next;
//             }
//             ptr->next = &new_node;
//         }
//     }

//     NodeList nextNode(){
//         if (head == nullptr){
//             return *head;
//         } else if (ind == nullptr || head != nullptr){
//             ind = head;
//             return *ind;
//         } else if (ind->next == nullptr){
//             ind = head;
//             return *ind;
//         } else {
//             ind = ind->next;
//             return *ind;
//         }
//     }

//     bool is_empty(){
//         return head == nullptr;
//     }
// };

class ExploreNode: public rclcpp::Node
{
public:
    ExploreNode(): Node("explore_node")
    {
        this->declare_parameter("range", 1.0);
        this->declare_parameter("time_scale", 1.0); // 1.16
        this->declare_parameter("time_duration", 7.0); 
        this->declare_parameter("distance", 7.0);

        this->distance_ = this->get_parameter("distance").as_double();
        this->time_scale_ = this->get_parameter("time_scale").as_double();
        this->time_duration_ = this->get_parameter("time_duration").as_double();
        this->range_ = this->get_parameter("range").as_double();
        
        this->count = 0;
        this->start_time_ = this->get_clock()->now();

         // min index and max index of the lidar data
        this->min_index = 270 + this->converter(-30.0);
        this->max_index = 270 + this->converter(30.0);

        // Define state from the start.
        this->state_ = "EXPLORE";
        this->reset_array();

        // Initialize link list
        // this->list = LinkedList();

        this->nav_pub_ = this->create_publisher<Twist>("/nav_cmd_vel", 10);
        this->goal_pub_ = this->create_publisher<TargetGoal>("/target_goal", 10);
        this->pose_text_pub_ = this->create_publisher<String>("/pose_text", 10);

        // Change later
        this->object_sub_ = this->create_subscription<ObjectInfo>("/markers", 10,
            std::bind(&ExploreNode::object_callback, this, _1));
            
        this->pose_sub_ = this->create_subscription<PoseWithCStamped>("/pose", 10,
            std::bind(&ExploreNode::pose_callback, this, _1));

        this->scan_sub_ = this->create_subscription<LaserScan>("/scan", 10,
            std::bind(&ExploreNode::scan_callback, this, _1));

        this->response_sub_ = this->create_subscription<String>("/response", 10,
            std::bind(&ExploreNode::response_callback, this, _1));

        this->point_sub_ = this->create_subscription<Int32>("/registered_point", 10,
            std::bind(&ExploreNode::register_callback, this, _1));

        this->turn_ = false;
        this->command_.linear.x = 0.0;
        this->command_.angular.z = 0.0;
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&ExploreNode::publish_cmd, this));

        this->timer_repeat_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&ExploreNode::explore_drive, this));

        this->test_ = this->create_wall_timer(std::chrono::seconds(5),
            std::bind(&ExploreNode::print_list, this));

        this->turn_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                    std::bind(&ExploreNode::turn, this));
        this->drive_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                    std::bind(&ExploreNode::drive_amount, this));

        this->turn_timer_->cancel();
        this->drive_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Explore node has been started.");
    }

private:

    void register_callback(const Int32::SharedPtr msg){
        if (msg->data < 9 && msg->data >= 0){
            this->count = msg->data;
        }
    }

    void reset_array(){
        for (int i = 0; i < 9; i++){
            this->arr[i].x = i;
            this->arr[i].y = i-1;
            this->arr[i].number = i;
        }
    }

    void response_callback(const String::SharedPtr msg){
        if (msg->data == "RECEIVED"){
            RCLCPP_INFO(this->get_logger(), "Navigating to object %d", this->count);
            if (this->count < 9){
                this->count += 1;
            } else {
                this->count = 0;
            }
        }
    }

    void print_list(){
        for (int i = 0; i < 9; i++){
            if (this->arr[i].number > 0){
                RCLCPP_INFO(this->get_logger(), "Object %d: %s", i, this->arr[i].description.c_str());
            }
        }
    }

    void object_callback(const ObjectInfo::SharedPtr msg){
        if (msg->number > 0 && msg->number < 10){
            this->arr[msg->number-1] = *msg;
        }
    }

    void pose_callback(const PoseWithCStamped::SharedPtr msg){
        this->pose_ = msg->pose.pose;
        String text;
        text.data = "X: " + std::to_string(this->pose_.position.x) + " Y: " + std::to_string(this->pose_.position.y);
        this->pose_text_pub_->publish(text);
    }

    // Constantly checking lidar data
    void scan_callback(const LaserScan::SharedPtr msg){
        auto ranges = msg->ranges;
        for (int i = this->min_index; i < this->max_index; i++){
            if (verify_lidar_data(*msg, ranges[i])){
                if (ranges[i] < this->range_ && ranges[i] > 0.1){
                    this->turn_ = true;
                    break;
                }
                else {
                    this->turn_ = false;
                }
            }
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
        return (int) (radian / 0.005817166529595852);
    }

    void publish_cmd(){
        if (this->state_ == "EXPLORE" || this->state_ == "TURN" || this->state_ == "DRIVE"){
            this->nav_pub_->publish(this->command_);
        }
    }

    void drive(double linear, double angular){
        this->command_.linear.x = linear;
        this->command_.angular.z = angular;
    }

    void turn(){
        RCLCPP_INFO(this->get_logger(), "Turning...");
        drive(0.0, 1.0);
        double duration = this->time_scale_ * 3.14;
        rclcpp::Duration dt = this->get_clock()->now() - this->turn_time_;
        this->publish_cmd();
        if (dt.seconds() > duration){
            this->turn_timer_->cancel();
            this->state_ = "DRIVE";
            this->drive_time_ = this->get_clock()->now();
            this->drive_timer_->reset();
            RCLCPP_INFO(this->get_logger(), "Turning completed.");
        }
    }

    void drive_amount(){
        drive(SPEED, 0.0);
        double duration = (abs(this->distance_) / SPEED);
        rclcpp::Duration dt = this->get_clock()->now() - this->drive_time_;
        this->publish_cmd();
        if (dt.seconds() > duration){
            this->drive_timer_->cancel();
            this->state_ = "EXPLORE";
        }
    }

    void explore_drive(){
        rclcpp::Duration duration_ = this->get_clock()->now() - this->start_time_;
        // RCLCPP_INFO(this->get_logger(), "Turn %d, and %.2f elapsed.", this->count, duration_.seconds() / 60.0);
        if (this->state_ == "EXPLORE"){
            RCLCPP_INFO(this->get_logger(), "Exploring...");
            if ((duration_.seconds() / 60.0 ) > this->time_duration_){
                this->state_ = "NAVIGATE";
                TargetGoal goal;
                goal.x = 0.0;
                goal.y = 0.0;
                goal.yaw = 0.0;
                this->goal_pub_->publish(goal);
            } 
            else if (this->turn_){
                drive(0.0, 1.0);
                count++;
            } else if (abs(this->pose_.position.x) > 6.5 || abs(this->pose_.position.y) > 6.5){
                // double distance;
                // if (abs(this->pose_.position.x) > 10.0){
                //     distance = abs(this->pose_.position.x) - 10.0;
                // } else {
                //     distance = (this->pose_.position.y) - 10.0;
                // }
                this->turn_time_ = this->get_clock()->now();
                this->state_ = "TURN";
                this->turn_timer_->reset();
                RCLCPP_INFO(this->get_logger(), "Turning...");
            }
            else {
                drive(SPEED, 0.0);
            }
        }
        else if (this->state_ == "NAVIGATE"){
            if (this->arr[this->count].number > 0){
                TargetGoal goal;
                goal.x = this->arr[this->count].x;
                goal.y = this->arr[this->count].y;
                goal.yaw = 0.0;
                this->goal_pub_->publish(goal);
            } else {
                this->count = (this->count + 1) % 9;
            }
        }
    }    

    rclcpp::Publisher<Twist>::SharedPtr nav_pub_;
    rclcpp::Publisher<TargetGoal>::SharedPtr goal_pub_;
    rclcpp::Subscription<ObjectInfo>::SharedPtr object_sub_;
    rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<String>::SharedPtr response_sub_;
    rclcpp::Publisher<String>::SharedPtr pose_text_pub_;
    rclcpp::TimerBase::SharedPtr timer_, timer_repeat_, test_, turn_timer_, drive_timer_;

    rclcpp::Subscription<PoseWithCStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<Int32>::SharedPtr point_sub_;

    std::string state_;
    int min_index, max_index;
    double range_, time_scale_, time_duration_, distance_;
    bool turn_, is_turning_, is_driving_;
    Twist command_;
    Pose pose_;
    rclcpp::Time start_time_, turn_time_, drive_time_;
    int count;
    ObjectInfo arr[9];
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExploreNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
