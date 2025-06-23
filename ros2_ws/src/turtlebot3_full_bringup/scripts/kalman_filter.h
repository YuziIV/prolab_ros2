#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>

class FilterNode : public rclcpp::Node
{
public:
    FilterNode();

private:
    void predict(double dt);
    void correct(const Eigen::VectorXd &z);
    void sensorCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>> sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::Time last_time_;

    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd A_ = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd H_ = Eigen::MatrixXd::Zero(4, 6);
    Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    Eigen::MatrixXd R_ = Eigen::MatrixXd::Identity(4, 4) * 0.05;

    bool initialized_ = false;
};
