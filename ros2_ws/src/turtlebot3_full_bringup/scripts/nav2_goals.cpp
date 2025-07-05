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

    bool send_goal(double x, double y, double qx, double qy, double qz, double qw) {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return false;
        }

        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;

        goal.pose.pose.orientation.x = qx;
        goal.pose.pose.orientation.y = qy;
        goal.pose.pose.orientation.z = qz;
        goal.pose.pose.orientation.w = qw;

        RCLCPP_INFO(this->get_logger(), "Sending goal to x=%.3f, y=%.3f, theta=%.3f", x, y, 
                    std::atan2(2.0 * (qy * qw + qx * qz), 1.0 - 2.0 * (qx * qx + qy * qy)));

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

    node->send_goal(-8.217, 3.917, 0.0, 0.0, 0.70146, 0.71271); // P1
    node->send_goal(-3.275, 8.484, 0.0, 0.0, 0.00688, 0.99998); // P2
    node->send_goal( 0.434, 6.646, 0.0, 0.0, -0.35987, 0.93300); // P3
    node->send_goal( 4.419, 3.689, 0.0, 0.0, -0.52888, 0.84870); // P4
    node->send_goal( 5.014, -2.817, 0.0, 0.0, -0.97538, 0.22054); // P5
    node->send_goal(-0.927, -6.156, 0.0, 0.0, -0.99050, 0.13753); // P6
    node->send_goal(-5.139, -6.104, 0.0, 0.0, 0.84870, 0.52887); // P7
    node->send_goal(-6.443, -1.051, 0.0, 0.0, 0.85546, 0.51788); // P8


    rclcpp::shutdown();
    return 0;
}