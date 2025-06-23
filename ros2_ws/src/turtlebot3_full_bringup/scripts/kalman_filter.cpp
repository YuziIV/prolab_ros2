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
    FilterNode() : Node("filter_node")
    {
        using namespace message_filters;

        odom_sub_.subscribe(this, "/odom");
        imu_sub_.subscribe(this, "/imu");

        sync_ = std::make_shared<TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>>(odom_sub_, imu_sub_, 10);
        sync_->registerCallback(std::bind(&FilterNode::sensorCallback, this, std::placeholders::_1, std::placeholders::_2));

        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/prediction", 10);

        // Setup observation matrix
        H_.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity(); // vx, vy
        H_.block<2, 2>(2, 4) = Eigen::Matrix2d::Identity(); // ax, ay
    }

private:
    void sensorCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received synchronized odom and imu");

        rclcpp::Time current_time = odom_msg->header.stamp;
        double dt = 0.01;

        if (!last_time_.nanoseconds() == 0)
            dt = (current_time - last_time_).seconds();

        last_time_ = current_time;

        // Transition Matrix A
        A_ = Eigen::MatrixXd::Identity(6, 6);
        A_(0, 2) = dt;
        A_(1, 3) = dt;
        A_(0, 4) = 0.5 * dt * dt;
        A_(1, 5) = 0.5 * dt * dt;
        A_(2, 4) = dt;
        A_(3, 5) = dt;

        // Prediction
        x_ = A_ * x_;
        P_ = A_ * P_ * A_.transpose() + Q_;

        double vx = odom_msg->twist.twist.linear.x;
        double vy = odom_msg->twist.twist.linear.y;
        double ax = imu_msg->linear_acceleration.x;
        double ay = imu_msg->linear_acceleration.y;

        Eigen::VectorXd z(4);
        z << vx, vy, ax, ay;

        // Correction
        Eigen::VectorXd y = z - H_ * x_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;

        // Publish prediction
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

    // Subscribers and synchronizer
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>> sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::Time last_time_;

    // Kalman filter variables
    Eigen::VectorXd x_ = Eigen::VectorXd::Zero(6);               // state vector
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(6, 6);        // state covariance
    Eigen::MatrixXd A_ = Eigen::MatrixXd::Identity(6, 6);        // transition matrix
    Eigen::MatrixXd H_ = Eigen::MatrixXd::Zero(4, 6);            // observation matrix
    Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01; // process noise
    Eigen::MatrixXd R_ = Eigen::MatrixXd::Identity(4, 4) * 0.05; // measurement noise
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}
