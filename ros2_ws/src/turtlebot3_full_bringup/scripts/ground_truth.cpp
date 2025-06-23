#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>

class GroundTruthPublisher : public rclcpp::Node
{
public:
    GroundTruthPublisher()
    : Node("ground_truth")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ground_truth_pose", 10);
        sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10,
            std::bind(&GroundTruthPublisher::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        auto it = std::find(msg->name.begin(), msg->name.end(), "turtlebot3_burger");
        if (it != msg->name.end())
        {
            size_t index = std::distance(msg->name.begin(), it);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->get_clock()->now();
            pose.header.frame_id = "odom";  // or "world" as needed
            pose.pose = msg->pose[index];
            pub_->publish(pose);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthPublisher>());
    rclcpp::shutdown();
    return 0;
}
