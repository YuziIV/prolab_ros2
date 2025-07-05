#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class nav : public rclcpp::Node {
public:
    nav() : Node("nav_node") {
        client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
    }

    bool send_goal(double x, double y, double theta) {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return false;
        }

        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.orientation.w = theta;

        RCLCPP_INFO(this->get_logger(), "Sending goal to x=%.3f, y=%.3f, theta=%.3f", x, y, theta);

        auto send_goal_future = client_ptr_->async_send_goal(goal);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
            return false;
        }

        auto goal_handle = send_goal_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
            return false;
        }

        auto result_future = client_ptr_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get result");
            return false;
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
            return true;
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        }
        return false;
    }


private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav>();

    rclcpp::sleep_for(std::chrono::seconds(5));  //some time for startup
    rclcpp::spin_some(node);

    rclcpp::sleep_for(std::chrono::seconds(2));  //some time for startup

    node->send_goal(-8.217, 3.917, 1.560);   // P1 
    node->send_goal(-3.275, 8.484, 0.014);   // P2 
    node->send_goal( 0.434, 6.646, -0.735);  // P3 
    node->send_goal( 4.419, 3.689, -1.124);  // P4 
    node->send_goal( 5.014, -2.817, -1.328); // P5  
    node->send_goal(-0.927, -6.156, -1.296); // P6 
    node->send_goal(-5.139, -6.104, 2.095);  // P7 
    node->send_goal(-6.443, -1.051, 2.032);  // P8


    rclcpp::shutdown();
    return 0;
}