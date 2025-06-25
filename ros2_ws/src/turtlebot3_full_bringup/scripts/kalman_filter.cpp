#include "kalman_filter.h"

FilterNode::FilterNode() : Node("filter_node")
{
    using namespace message_filters;

    odom_sub_.subscribe(this, "/odom");
    imu_sub_.subscribe(this, "/imu");

    sync_ = std::make_shared<TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>>(odom_sub_, imu_sub_, 10);
    sync_->registerCallback(std::bind(&FilterNode::sensorCallback, this, std::placeholders::_1, std::placeholders::_2));

    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/prediction", 10);

    H_.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity(); // vx, vy
    H_.block<2, 2>(2, 4) = Eigen::Matrix2d::Identity(); // ax, ay
}

void FilterNode::predict(double dt)
{
    A_(0, 2) = dt;
    A_(1, 3) = dt;
    A_(0, 4) = 0.5 * dt * dt;
    A_(1, 5) = 0.5 * dt * dt;
    A_(2, 4) = dt;
    A_(3, 5) = dt;

    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void FilterNode::correct(const Eigen::VectorXd &z)
{
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;
}

void FilterNode::sensorCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
    RCLCPP_INFO(this->get_logger(), "Received synchronized odom and imu");

    rclcpp::Time current_time = odom_msg->header.stamp;
    double dt = 0.01;

    if (!initialized_)
    {
        x_.setZero();
        x_(0) = odom_msg->pose.pose.position.x;
        x_(1) = odom_msg->pose.pose.position.y;
        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Initialized Kalman filter at x=%.2f, y=%.2f", x_(0), x_(1));
        last_time_ = current_time;
        return;
    }

    if (last_time_.nanoseconds() != 0)
        dt = (current_time - last_time_).seconds();

    last_time_ = current_time;

    predict(dt);

    auto q = odom_msg->pose.pose.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    double vx_robot = odom_msg->twist.twist.linear.x;
    double vy_robot = odom_msg->twist.twist.linear.y;
    double ax_robot = imu_msg->linear_acceleration.x;
    double ay_robot = imu_msg->linear_acceleration.y;

    double vx_world = std::cos(yaw) * vx_robot - std::sin(yaw) * vy_robot;
    double vy_world = std::sin(yaw) * vx_robot + std::cos(yaw) * vy_robot;
    double ax_world = std::cos(yaw) * ax_robot - std::sin(yaw) * ay_robot;
    double ay_world = std::sin(yaw) * ax_robot + std::cos(yaw) * ay_robot;

    Eigen::VectorXd z(4);
    z << vx_world, vy_world, ax_world, ay_world;

    correct(z);

    geometry_msgs::msg::PoseWithCovarianceStamped output;
    output.header.stamp = current_time;
    output.header.frame_id = "odom";
    output.pose.pose.position.x = x_(0);
    output.pose.pose.position.y = x_(1);
    output.pose.pose.position.z = 0.0;
    output.pose.pose.orientation = imu_msg->orientation;

    for (int i = 0; i < 6; ++i)
        output.pose.covariance[i * 6 + i] = P_(i, i);

    pub_->publish(output);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}
