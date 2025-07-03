#pragma once
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class FilterNode : public rclcpp::Node
{
public:
    FilterNode();

private:
    // Kalman Filter state and covariance
    Eigen::VectorXd mu_;        // State vector (x, y, theta, vx, vy, wz)
    Eigen::MatrixXd Sigma_;     // Covariance matrix
    Eigen::MatrixXd Q_;         // Process noise covariance 
    Eigen::MatrixXd R_imu_;     // Measurement noise covariance

    rclcpp::Time last_time_;    // Last time stamp for prediction
    bool first_measurement_received_;

    // ROS Subscriptions and Publisher
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>> sync_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;

    // Methods for prediction and correction
    void predict(double dt, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    void correct(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

    // Callback for synchronized sensor data
    void sensorCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

    // Helper to publish the current state
    void publishEstimatedPose(const rclcpp::Time& stamp);
};
