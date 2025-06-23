// This ROS2 C++ node subscribes to Gazebo's /gazebo/model_states topic,
// extracts the pose of a specified model, and publishes it as a
// geometry_msgs/msg/PoseStamped message for RViz2 visualization.

#include <chrono>       // For std::chrono::duration and related types
#include <functional>   // For std::bind
#include <memory>       // For std::shared_ptr
#include <string>       // For std::string

#include "rclcpp/rclcpp.hpp"                      // ROS2 C++ client library
#include "gazebo_msgs/msg/model_states.hpp"       // Message type for /gazebo/model_states
#include "geometry_msgs/msg/pose_stamped.hpp"     // Message type for publishing pose to RViz2

using namespace std::chrono_literals; // For using literals like 500ms

// Define a class for our Ground Truth Publisher node
class GroundTruthPublisher : public rclcpp::Node
{
public:
  // Constructor for the GroundTruthPublisher node
  GroundTruthPublisher()
  : rclcpp::Node("ground_truth_publisher"),  // Initialize the node with the name "ground_truth_publisher"
    model_name_("robot"),                    // Default model name to look for
    frame_id_("odom")                       // Default frame ID for the published pose
  {
    // Declare parameters for the model name and frame ID.
    // These can be overridden via command line or YAML files.
    this->declare_parameter<std::string>("model_name", model_name_);
    this->declare_parameter<std::string>("frame_id", frame_id_);

    // Get the parameter values (if provided, otherwise use defaults)
    this->get_parameter("model_name", model_name_);
    this->get_parameter("frame_id", frame_id_);

    RCLCPP_INFO(this->get_logger(), "GroundTruthPublisher node started.");
    RCLCPP_INFO(this->get_logger(), "Looking for model: '%s'", model_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing pose in frame: '%s'", frame_id_.c_str());

    // Create a subscriber to the /gazebo/model_states topic.
    // The queue size of 10 means it will buffer up to 10 messages if processing is slow.
    // std::bind is used to tie the callback function to this object's context.
    subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/gazebo/model_states", 10, std::bind(&GroundTruthPublisher::model_states_callback, this, std::placeholders::_1));

    // Create a publisher for the geometry_msgs/msg/PoseStamped message.
    // The topic name will be "/ground_truth/pose".
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ground_truth/pose", 10);
  }

private:
  // Callback function for when a new ModelStates message is received
  void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    // Iterate through the names of the models in the received message
    for (size_t i = 0; i < msg->name.size(); ++i) {
      // Check if the current model name matches our target model name
      if (msg->name[i] == model_name_) {
        // If a match is found, create a new PoseStamped message
        auto pose_stamped_msg = geometry_msgs::msg::PoseStamped();

        // Set the timestamp of the header to the current ROS time
        pose_stamped_msg.header.stamp = this->now();
        // Set the frame ID for the header
        pose_stamped_msg.header.frame_id = frame_id_;

        // Copy the pose data from the Gazebo message to our PoseStamped message
        pose_stamped_msg.pose = msg->pose[i];

        // Publish the PoseStamped message
        publisher_->publish(pose_stamped_msg);

        // Log that the pose was published
        // RCLCPP_INFO(this->get_logger(), "Published pose for '%s'.", model_name_.c_str());
        return; // Exit the loop and function once the model is found and its pose published
      }
    }
    // If the loop completes without finding the model, log a warning
    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Model '%s' not found in Gazebo states.", model_name_.c_str());
  }

  // Private member variables
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_; // Subscriber object
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;    // Publisher object
  std::string model_name_;  // Name of the model to track
  std::string frame_id_;    // Frame ID for the published pose
};

// Main function where the node is initialized and spun
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize ROS2
  // Create an instance of our GroundTruthPublisher node and spin it
  // Spinning the node keeps it alive and processes callbacks
  rclcpp::spin(std::make_shared<GroundTruthPublisher>());
  rclcpp::shutdown(); // Shutdown ROS2 when the node stops
  return 0;
}
