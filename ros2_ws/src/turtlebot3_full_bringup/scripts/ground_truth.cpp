// This ROS2 C++ node subscribes to Gazebo's /gazebo/model_states topic.
// It specifically extracts the pose of the model named "burger" from the incoming message,
// and then publishes this individual pose as a geometry_msgs/msg/PoseStamped message
// to the topic /burger/ground_truth_pose.
// The published pose's frame_id is explicitly set to "map" to align with RViz2's global frame,
// assuming Gazebo's world origin is equivalent to the RViz2 map origin.

#include <chrono>       // For std::chrono::duration and related types
#include <functional>   // For std::bind
#include <memory>       // For std::shared_ptr
#include <string>       // For std::string

#include "rclcpp/rclcpp.hpp"                      // ROS2 C++ client library
#include "gazebo_msgs/msg/model_states.hpp"       // Message type for /gazebo/model_states
#include "geometry_msgs/msg/pose_stamped.hpp"     // Message type for publishing pose to RViz2

using namespace std::chrono_literals; // For using literals like 500ms

// Define a class for our Burger Pose Extractor node
class BurgerPoseExtractor : public rclcpp::Node
{
public:
  // Constructor for the BurgerPoseExtractor node
  BurgerPoseExtractor()
  : rclcpp::Node("burger_pose_extractor"),  // Initialize the node with the name "burger_pose_extractor"
    target_model_name_("burger"),           // The specific model name to look for
    // IMPORTANT CHANGE: Set default output_frame_id_ to "map"
    output_frame_id_("map")                 // Frame ID for the published PoseStamped message
  {
    // Declare parameters so they can be overridden via command line or YAML files.
    this->declare_parameter<std::string>("target_model_name", target_model_name_);
    this->declare_parameter<std::string>("output_frame_id", output_frame_id_);

    // Get the parameter values (if provided, otherwise use defaults)
    this->get_parameter("target_model_name", target_model_name_);
    this->get_parameter("output_frame_id", output_frame_id_);

    RCLCPP_INFO(this->get_logger(), "BurgerPoseExtractor node started.");
    RCLCPP_INFO(this->get_logger(), "Looking for model: '%s' on /model_states", target_model_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing its pose to /burger/ground_truth_pose in frame: '%s'", output_frame_id_.c_str());

    // Create a subscriber to the /gazebo/model_states topic.
    subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states",
      10, // QoS history depth
      std::bind(&BurgerPoseExtractor::model_states_callback, this, std::placeholders::_1)
    );

    // Create a publisher for the geometry_msgs/msg/PoseStamped message.
    // The topic name will be "/burger/ground_truth_pose".
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/burger/ground_truth_pose", 10);
  }

private:
  // Callback function for when a new ModelStates message is received
  void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    // Iterate through the names of the models in the received message
    for (size_t i = 0; i < msg->name.size(); ++i) {
      // Check if the current model name matches our target model name ("burger")
      if (msg->name[i] == target_model_name_) {
        // If a match is found, create a new PoseStamped message
        auto pose_stamped_msg = geometry_msgs::msg::PoseStamped();

        // Set the timestamp of the header to the current ROS time.
        // This is important for RViz2 visualization time synchronization.
        pose_stamped_msg.header.stamp = this->now();
        
        // Set the frame ID for the header to the configured output_frame_id_ (now "map").
        // This tells RViz2 that the pose is relative to the "map" frame.
        pose_stamped_msg.header.frame_id = output_frame_id_;

        // Copy the pose data from the Gazebo ModelStates message to our PoseStamped message
        pose_stamped_msg.pose = msg->pose[i];

        // Publish the PoseStamped message
        publisher_->publish(pose_stamped_msg);

        // Exit the loop and function once the model is found and its pose published.
        return;
      }
    }
  }

  // Private member variables
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_; // Subscriber object for ModelStates
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;    // Publisher object for PoseStamped
  std::string target_model_name_;  // Name of the specific model to track (e.g., "burger")
  std::string output_frame_id_;    // Frame ID for the published pose (e.g., "map")
};

// Main function where the node is initialized and spun
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize ROS2
  // Create an instance of our BurgerPoseExtractor node and spin it
  rclcpp::spin(std::make_shared<BurgerPoseExtractor>());
  rclcpp::shutdown(); // Shutdown ROS2 when the node stops
  return 0;
}
