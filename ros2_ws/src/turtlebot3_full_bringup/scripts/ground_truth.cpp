#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GroundTruthPublisher : public rclcpp::Node
{
public:
    GroundTruthPublisher()
    : Node("ground_truth_publisher")
    {
        // Publisher
        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ground_truth", 10);

        // Subscriber
        sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10,
            std::bind(&GroundTruthPublisher::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        if (msg->name.empty())
            return;

        size_t last_index = msg->name.size() - 1;
        const auto & pose = msg->pose[last_index];
        const std::string & name = msg->name[last_index];

        geometry_msgs::msg::PoseStamped stamped_pose;
        stamped_pose.header.stamp = this->now();
        stamped_pose.header.frame_id = "map";  // or "world" if that fits your TF tree
        stamped_pose.pose = pose;

        pub_->publish(stamped_pose);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[%s] x: %.2f, y: %.2f", name.c_str(), pose.position.x, pose.position.y);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthPublisher>());
    rclcpp::shutdown();
    return 0;
}
