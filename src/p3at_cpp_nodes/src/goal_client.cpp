#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "p3at_interface/msg/target_goal.hpp"
#include "std_msgs/msg/string.hpp"

using String = std_msgs::msg::String;
using TargetGoal = p3at_interface::msg::TargetGoal;
using Quaternion = tf2::Quaternion;
using Trigger = std_srvs::srv::Trigger;
using Pose = geometry_msgs::msg::Pose;
using Nav2Pose = nav2_msgs::action::NavigateToPose;
using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Pose>;
using namespace std::placeholders;

class GoalClientNode: public rclcpp::Node
{
public:
    GoalClientNode(): Node("goal_test")
    {
        // this->declare_parameter("goal_x", 5.0);
        // this->declare_parameter("goal_y", 5.0);

        // double goal_x, goal_y;
        // goal_x = this->get_parameter("goal_x").as_double();
        // goal_y = this->get_parameter("goal_y").as_double();

        this->goal_client_ = rclcpp_action::create_client<Nav2Pose>(
            this,
            "navigate_to_pose"
        );

        this->cancel_service_ = this->create_service<Trigger>(
            "cancel_goal",
            std::bind(&GoalClientNode::callbackStopCurrentGoal, this, _1, _2));

        this->response_pub_ = this->create_publisher<String>("response", 10);
        this->goal_sub_ = this->create_subscription<TargetGoal>(
            "target_goal",
            10,
            std::bind(&GoalClientNode::goal_callback, this, _1)
        );
        this->is_naving_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal Register has been started.");
    }

private:

    void goal_callback(const TargetGoal::SharedPtr msg){
        if (this->is_naving_){
            this->text_.data = "WAIT";
            this->response_pub_->publish(this->text_);
            RCLCPP_INFO(this->get_logger(), "Wait for current goal to finish or cancel it.");
        } else {
            Pose target;
            target.position.x = msg->x;
            target.position.y = msg->y;

            Quaternion q;
            q.setRPY(0, 0, msg->yaw);
            target.orientation.x = q.x();
            target.orientation.y = q.y();
            target.orientation.z = q.z();
            target.orientation.w = q.w();
            this->text_.data = "RECEIVED";
            this->response_pub_->publish(this->text_);            
            send_goal(target);
        }
    }
    
    void callbackStopCurrentGoal(const std::shared_ptr<Trigger::Request> request,
                                const std::shared_ptr<Trigger::Response> response){
        (void) request;
        if (this->is_naving_){
            RCLCPP_INFO(this->get_logger(), "Canceling goal...");
            response.get()->success = true;
            response.get()->message = "Goal canceled";
            this->is_naving_ = false;
            this->goal_client_->async_cancel_goal(this->goal_handle_);
        } else {
            RCLCPP_INFO(this->get_logger(), "No goal to cancel");
            response.get()->success = false;
            response.get()->message = "No Goal at the moment!";
        }
    }

    void send_goal(Pose target){
        // Wait for action server 
        while (!this->goal_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server to be up...");
        }

        // Create a goal
        auto goal = Nav2Pose::Goal();
        goal.pose.pose = target;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();

        auto options = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
        options.result_callback =
            std::bind(&GoalClientNode::result_callback, this, _1);
        options.feedback_callback =
            std::bind(&GoalClientNode::feedback_callback, this, _1, _2);
        options.goal_response_callback =
            std::bind(&GoalClientNode::response_callback, this, _1);

        // Send goal
        RCLCPP_INFO(this->get_logger(), "Sending goal...");
        this->goal_client_->async_send_goal(goal, options);

    }

    // Goal Response callback
    void response_callback(const Nav2GoalHandle::SharedPtr &goal_handle){
        if (!goal_handle){
            RCLCPP_INFO(this->get_logger(), "Goal was rejected by the server");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the server");
            this->goal_handle_ = goal_handle;
            this->is_naving_ = true;
        }
    }

    // Feedback callback
    void feedback_callback(const Nav2GoalHandle::SharedPtr &goal_handle,
                        const std::shared_ptr<const Nav2Pose::Feedback> feedback){
        (void) goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received feedback: %f", feedback->distance_remaining);
    }

    // Result callback
    void result_callback(const Nav2GoalHandle::WrappedResult &result){
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED){
            RCLCPP_INFO(this->get_logger(), "Goal aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED){
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Unknown result code");
        }
        this->is_naving_ = false;
    }

    rclcpp_action::Client<Nav2Pose>::SharedPtr goal_client_;
    rclcpp::Service<Trigger>::SharedPtr cancel_service_;
    
    rclcpp::Subscription<TargetGoal>::SharedPtr goal_sub_;
    rclcpp::Publisher<String>::SharedPtr response_pub_;

    Nav2GoalHandle::SharedPtr goal_handle_;
    bool is_naving_;
    Pose goal_test_;
    String text_; // for displaying reponse of naving
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}