#include "kalman_filter.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For converting Quaternion to RPY
#include <cmath> // For std::cos, std::sin, std::fmod

// Constructor
FilterNode::FilterNode() : Node("filter_node")
{
    using namespace message_filters;

    // ROS Subscriptions
    odom_sub_.subscribe(this, "/odom");
    imu_sub_.subscribe(this, "/imu");

    // Time Synchronizer
    sync_ = std::make_shared<TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>>(odom_sub_, imu_sub_, 10);
    sync_->registerCallback(std::bind(&FilterNode::sensorCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Publisher for the estimated pose
    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/KF_pose", 10);

    // Initialize state and covariance
    mu_ = Eigen::VectorXd::Zero(6); // mu_ represents mu_t-1 initially
    Sigma_ = Eigen::MatrixXd::Identity(6, 6) * 1e-9; // Sigma_ represents Sigma_t-1 initially

    // Initialize process noise covariance (Rt from algorithm, Q_ in code)
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    Q_(0, 0) = 0.05; // x
    Q_(1, 1) = 0.05; // y
    Q_(2, 2) = 0.02; // theta
    Q_(3, 3) = 0.1;  // vx
    Q_(4, 4) = 0.1;  // vy
    Q_(5, 5) = 0.05; // wz

    // Initialize measurement noise covariance (Qt from algorithm, R_imu_ in code)
    R_imu_ = Eigen::MatrixXd::Identity(3, 3);
    R_imu_(0, 0) = 0.01; // wz (gyro_z) noise
    R_imu_(1, 1) = 0.1;  // ax (accel_x) noise - Assumed to measure vx
    R_imu_(2, 2) = 0.1;  // ay (accel_y) noise - Assumed to measure vy

    last_time_ = this->now();
    first_measurement_received_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Linear Kalman Filter Node Initialized!");
}

// Prediction Step (LKF implementation)
void FilterNode::predict(double dt, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    // --- Define Control Input ut ---
    // In our LKF, the odometry velocities are the control inputs.
    Eigen::VectorXd ut = Eigen::VectorXd(3);
    ut << odom_msg->twist.twist.linear.x,    // local_vx
          odom_msg->twist.twist.linear.y,    // local_vy
          odom_msg->twist.twist.angular.z;   // angular_z_odom

    // --- Define State Transition Matrix At ---
    Eigen::MatrixXd At = Eigen::MatrixXd::Identity(6, 6);
    At(0, 3) = dt; // x_t = x_t-1 + vx_t-1 * dt
    At(1, 4) = dt; // y_t = y_t-1 + vy_t-1 * dt
    At(2, 5) = dt; // theta_t = theta_t-1 + wz_t-1 * dt
    

    At(3, 3) = 0.0;
    At(4, 4) = 0.0;
    At(5, 5) = 0.0;

    // --- Define Control Input Matrix Bt ---
    Eigen::MatrixXd Bt = Eigen::MatrixXd::Zero(6, 3);
    Bt(3, 0) = 1.0; // vx_t = v_x_odom
    Bt(4, 1) = 1.0; // vy_t = v_y_odom
    Bt(5, 2) = 1.0; // wz_t = w_z_odom

    // --- Prediction for State
    mu_ = At * mu_ + Bt * ut; // mu_ now holds ¯μt

    // Normalize theta to [-PI, PI]
    mu_(2) = std::fmod(mu_(2), 2 * M_PI);
    if (mu_(2) > M_PI) mu_(2) -= 2 * M_PI;
    if (mu_(2) < -M_PI) mu_(2) += 2 * M_PI;

    // --- Predict Covariance
    Sigma_ = At * Sigma_ * At.transpose() + Q_; // Sigma_ now holds ¯Σt

    RCLCPP_INFO(this->get_logger(), "Prediction: x=%.2f, y=%.2f, theta=%.2f, vx=%.2f, vy=%.2f, wz=%.2f", mu_(0), mu_(1), mu_(2), mu_(3), mu_(4), mu_(5));
}


// Correction Step (LKF implementation using IMU data as measurement)
void FilterNode::correct(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
    // --- Extract IMU measurements (zt) ---
    double measured_wz = imu_msg->angular_velocity.z;
    double measured_vx_from_imu = imu_msg->linear_acceleration.x; // Treating ax as noisy vx measurement
    double measured_vy_from_imu = imu_msg->linear_acceleration.y; // Treating ay as noisy vy measurement

    Eigen::VectorXd zt = Eigen::VectorXd(3);
    zt << measured_wz, measured_vx_from_imu, measured_vy_from_imu;

    // Ct (Measurement Matrix)
    Eigen::MatrixXd Ct = Eigen::MatrixXd::Zero(3, 6);
    Ct(0, 5) = 1.0; // wz 
    Ct(1, 3) = 1.0; // vx 
    Ct(2, 4) = 1.0; // vy 

    // Qt (Measurement Noise Covariance)
    Eigen::MatrixXd Qt_measurement_noise = Eigen::MatrixXd::Identity(3, 3);
    Qt_measurement_noise(0, 0) = R_imu_(0, 0); // Noise for wz
    Qt_measurement_noise(1, 1) = R_imu_(1, 1); // Noise for vx (using R_imu_ for ax)
    Qt_measurement_noise(2, 2) = R_imu_(2, 2); // Noise for vy (using R_imu_ for ay)

    // --- Calculate Kalman Gain 
    Eigen::MatrixXd S = Ct * Sigma_ * Ct.transpose() + Qt_measurement_noise;
    Eigen::MatrixXd Kt = Sigma_ * Ct.transpose() * S.inverse();

    // --- Update State 
    mu_ = mu_ + Kt * (zt - Ct * mu_); // mu_ now holds μt

    // --- Update Covariance 
    Sigma_ = (Eigen::MatrixXd::Identity(6, 6) - Kt * Ct) * Sigma_; // Sigma_ now holds Σt

    // Normalize theta again after correction
    mu_(2) = std::fmod(mu_(2), 2 * M_PI);
    if (mu_(2) > M_PI) mu_(2) -= 2 * M_PI;
    if (mu_(2) < -M_PI) mu_(2) += 2 * M_PI;

    RCLCPP_INFO(this->get_logger(), "Correction: wz=%.2f, K(0)=%.2e, K(1)=%.2e, K(2)=%.2e", measured_wz, Kt(0,0), Kt(1,0), Kt(2,0));
}


// Callback for synchronized sensor data
void FilterNode::sensorCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                                const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
    if (!first_measurement_received_) {
        // Initialize state (μt-1) with first odometry reading
        mu_(0) = odom_msg->pose.pose.position.x;
        mu_(1) = odom_msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        mu_(2) = yaw;

        mu_(3) = odom_msg->twist.twist.linear.x;
        mu_(4) = odom_msg->twist.twist.linear.y;
        mu_(5) = odom_msg->twist.twist.angular.z;

        last_time_ = odom_msg->header.stamp;
        first_measurement_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Initialized Kalman Filter state with first odometry.");
        return; // Skip prediction/correction for the very first message
    }

    rclcpp::Time current_time = odom_msg->header.stamp;
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "dt is zero or negative (%.4f). Skipping update.", dt);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received synchronized odom and imu. dt: %.4f", dt);

    predict(dt, odom_msg); // mu_ and Sigma_ become ¯μt and ¯Σt after this call

    correct(imu_msg); // mu_ and Sigma_ become μt and Σt after this call

    publishEstimatedPose(current_time);
}

// Helper to publish the current estimated pose
void FilterNode::publishEstimatedPose(const rclcpp::Time& stamp)
{
    geometry_msgs::msg::PoseWithCovarianceStamped output_pose;
    output_pose.header.stamp = stamp;
    output_pose.header.frame_id = "map"; // Or your desired fixed frame

    output_pose.pose.pose.position.x = mu_(0);
    output_pose.pose.pose.position.y = mu_(1);
    output_pose.pose.pose.position.z = 0.0; // 2D robot

    tf2::Quaternion q;
    q.setRPY(0, 0, mu_(2)); // Roll, Pitch are 0 for 2D
    output_pose.pose.pose.orientation.x = q.x();
    output_pose.pose.pose.orientation.y = q.y();
    output_pose.pose.pose.orientation.z = q.z();
    output_pose.pose.pose.orientation.w = q.w();

    // Reset covariance to zero to avoid artifacts from previous mappings
    for(int i = 0; i < 36; ++i) {
        output_pose.pose.covariance[i] = 0.0;
    }
    output_pose.pose.covariance[0] = Sigma_(0,0);
    output_pose.pose.covariance[1] = Sigma_(0,1);
    output_pose.pose.covariance[5] = Sigma_(0,2);
    output_pose.pose.covariance[6] = Sigma_(1,0);
    output_pose.pose.covariance[7] = Sigma_(1,1);
    output_pose.pose.covariance[11] = Sigma_(1,2);
    output_pose.pose.covariance[30] = Sigma_(2,0);
    output_pose.pose.covariance[31] = Sigma_(2,1);
    output_pose.pose.covariance[35] = Sigma_(2,2);
    pub_->publish(output_pose);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}