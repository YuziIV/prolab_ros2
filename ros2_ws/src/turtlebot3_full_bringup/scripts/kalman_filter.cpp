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

    // Initialize process noise covariance (Qt)
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    Q_(0, 0) = 0.05; // x
    Q_(1, 1) = 0.05; // y
    Q_(2, 2) = 0.02; // theta
    Q_(3, 3) = 0.1;  // vx
    Q_(4, 4) = 0.1;  // vy
    Q_(5, 5) = 0.05; // wz

    // Initialize measurement noise covariance (R)
    R_imu_ = Eigen::MatrixXd::Identity(3, 3);
    R_imu_(0, 0) = 0.01; // wz (gyro_z) noise
    R_imu_(1, 1) = 0.1;  // ax (accel_x) noise - Assumed to measure vx
    R_imu_(2, 2) = 0.1;  // ay (accel_y) noise - Assumed to measure vy

    last_time_ = this->now();
    first_measurement_received_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Kalman Filter Node Initialized!");
}

// Prediction Step (EKF approach for non-linear motion model)
void FilterNode::predict(double dt, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    // --- Extract local velocities from odometry ---
    // The odometry message provides velocities in the robot's local frame (e.g., base_link).
    double local_vx = odom_msg->twist.twist.linear.x;
    double local_vy = odom_msg->twist.twist.linear.y;
    double angular_z_odom = odom_msg->twist.twist.angular.z;

    // Get the orientation from the previous state (μt-1)
    // mu_ holds the state from the previous timestep before this prediction.
    double theta_prev = mu_(2);

    // --- Non-Linear Prediction for State (g(μt-1, ut)) ---
    // We calculate the predicted state ¯μt based on the previous state and control inputs.
    // This is Algorithm Line 1 of the EKF.
    
    // Transform local velocities to global velocities using the previous orientation
    double global_vx = local_vx * std::cos(theta_prev) - local_vy * std::sin(theta_prev);
    double global_vy = local_vx * std::sin(theta_prev) + local_vy * std::cos(theta_prev);

    // Create a temporary vector for the predicted state ¯μt
    Eigen::VectorXd mu_bar = Eigen::VectorXd::Zero(6);

    // Predict new state based on OLD state and new GLOBAL velocities
    // Note: The new velocities are not based on the old velocities, they are direct inputs.
    mu_bar(0) = mu_(0) + mu_(3) * dt; // x_t = x_t-1 + vx_t-1 * dt
    mu_bar(1) = mu_(1) + mu_(4) * dt; // y_t = y_t-1 + vy_t-1 * dt
    mu_bar(2) = mu_(2) + mu_(5) * dt; // theta_t = theta_t-1 + wz_t-1 * dt
    mu_bar(3) = global_vx;           // vx_t = transformed local vx
    mu_bar(4) = global_vy;           // vy_t = transformed local vy
    mu_bar(5) = angular_z_odom;      // wz_t = odom wz

    // Update the main state vector to the predicted state ¯μt
    mu_ = mu_bar;

    // Normalize theta to [-PI, PI]
    mu_(2) = std::fmod(mu_(2), 2 * M_PI);
    if (mu_(2) > M_PI) mu_(2) -= 2 * M_PI;
    if (mu_(2) < -M_PI) mu_(2) += 2 * M_PI;

    // --- Linearize Motion Model (Calculate Jacobian Gt) ---
    // Since the prediction g(μ, u) is non-linear (depends on theta), we must linearize it
    // by taking the partial derivative of g with respect to the state μ.
    // Gt = ∂g / ∂μ | at μt-1
    
    Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(6, 6);
    // Partial derivatives for x, y, theta predictions
    Gt(0, 3) = dt;
    Gt(1, 4) = dt;
    Gt(2, 5) = dt;
    
    // Partial derivatives for vx, vy predictions with respect to theta
    // ∂(vx_pred)/∂(theta_prev) = -local_vx*sin(theta_prev) - local_vy*cos(theta_prev)
    // ∂(vy_pred)/∂(theta_prev) =  local_vx*cos(theta_prev) - local_vy*sin(theta_prev)
    Gt(3, 2) = -local_vx * std::sin(theta_prev) - local_vy * std::cos(theta_prev);
    Gt(4, 2) =  local_vx * std::cos(theta_prev) - local_vy * std::sin(theta_prev);

    // The velocities are overwritten by the control input, so their dependence on previous velocities is zero.
    Gt(3, 3) = 0.0;
    Gt(4, 4) = 0.0;
    Gt(5, 5) = 0.0;

    // --- Predict Covariance (¯Σt) ---
    // Algorithm line 2 of EKF: ¯Σt = Gt Σt−1 Gt^T + Qt
    // Sigma_ is Sigma_t-1 when entering predict, becomes ¯Sigma_t after this line
    Sigma_ = Gt * Sigma_ * Gt.transpose() + Q_;

    RCLCPP_INFO(this->get_logger(), "Prediction: x=%.2f, y=%.2f, theta=%.2f, vx=%.2f, vy=%.2f, wz=%.2f", mu_(0), mu_(1), mu_(2), mu_(3), mu_(4), mu_(5));
}


// Correction Step (uses IMU data as measurement)
void FilterNode::correct(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
    // --- Extract IMU measurements (zt) ---
    // For a PURE LINEAR KF, we must assume these IMU measurements (linear_x, linear_y)
    // are directly measuring the GLOBAL linear velocities (vx, vy) of the robot,
    // and angular_velocity.z is directly measuring wz.
    // This is a significant simplification and likely unrealistic for a real IMU
    // on a turning robot, as IMU linear accelerations are in the body frame.

    double measured_wz = imu_msg->angular_velocity.z;
    // Assuming linear_acceleration.x/y from IMU directly measures vx/vy from the state.
    // This is an unrealistic assumption for a real IMU, which measures acceleration in its body frame.
    // But it makes the measurement model linear for a pure KF.
    double measured_vx_from_imu = imu_msg->linear_acceleration.x; // Treating ax as noisy vx measurement
    double measured_vy_from_imu = imu_msg->linear_acceleration.y; // Treating ay as noisy vy measurement

    // zt (Measurement vector) = [wz_imu, vx_imu, vy_imu]^T
    // The order here must match the order of rows in Ct
    Eigen::VectorXd zt = Eigen::VectorXd(3);
    zt << measured_wz, measured_vx_from_imu, measured_vy_from_imu;

    // Ct (Measurement Matrix)
    // Maps the 6-dimensional state [x, y, theta, vx, vy, wz]^T to the 3-dimensional measurement [wz, vx, vy]^T
    Eigen::MatrixXd Ct = Eigen::MatrixXd::Zero(3, 6);
    Ct(0, 5) = 1.0; // We measure wz (state index 5)
    Ct(1, 3) = 1.0; // We measure vx (state index 3)
    Ct(2, 4) = 1.0; // We measure vy (state index 4)

    // Qt (Measurement Noise Covariance) from algorithm line 3
    // This corresponds to R_imu_ in your code.
    Eigen::MatrixXd Qt_measurement_noise = Eigen::MatrixXd::Identity(3, 3);
    Qt_measurement_noise(0, 0) = R_imu_(0, 0); // Noise for wz
    Qt_measurement_noise(1, 1) = R_imu_(1, 1); // Noise for vx (using R_imu_ for ax)
    Qt_measurement_noise(2, 2) = R_imu_(2, 2); // Noise for vy (using R_imu_ for ay)

    // --- Calculate Kalman Gain (Kt) ---
    // Algorithm line 3: Kt = ¯Σt CTt (Ct ¯Σt CTt + Qt)−1
    // Note: Sigma_ at this point holds ¯Sigma_t
    Eigen::MatrixXd S = Ct * Sigma_ * Ct.transpose() + Qt_measurement_noise;
    Eigen::MatrixXd Kt = Sigma_ * Ct.transpose() * S.inverse();

    // --- Update State (μt) ---
    // Algorithm line 4: μt = ¯μt + Kt(zt − Ct ¯μt)
    // Note: mu_ at this point holds ¯μt
    mu_ = mu_ + Kt * (zt - Ct * mu_); // mu_ now holds μt

    // --- Update Covariance (Σt) ---
    // Algorithm line 5: Σt = (I − Kt Ct) ¯Σt
    // Note: Sigma_ at this point holds ¯Σt
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

        // The first odom velocities are local, transform them to global for the initial state
        double local_vx_init = odom_msg->twist.twist.linear.x;
        double local_vy_init = odom_msg->twist.twist.linear.y;
        double global_vx_init = local_vx_init * std::cos(yaw) - local_vy_init * std::sin(yaw);
        double global_vy_init = local_vx_init * std::sin(yaw) + local_vy_init * std::cos(yaw);

        mu_(3) = global_vx_init;
        mu_(4) = global_vy_init;
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

    // Fill covariance matrix (flattened for ROS message)
    // ROS covariance layout (row-major 6x6):
    // [ x-x, x-y, x-z, x-roll, x-pitch, x-yaw,
    //   y-x, y-y, y-z, y-roll, y-pitch, y-yaw,
    //   ...
    //   yaw-x, yaw-y, yaw-z, yaw-roll, yaw-pitch, yaw-yaw ]
    //
    // Our state has: [x, y, theta, vx, vy, wz]
    // We map (x, y, theta) from our Sigma_ to (x, y, yaw) in the ROS message.
    // The ROS message is for POSE covariance, not velocity covariance.

    // x-x (0,0)
    output_pose.pose.covariance[0] = Sigma_(0,0);
    // x-y (0,1)
    output_pose.pose.covariance[1] = Sigma_(0,1);
    // x-yaw (0,5) in ROS, corresponds to x-theta (0,2) in our state
    output_pose.pose.covariance[5] = Sigma_(0,2);

    // y-x (6,0)
    output_pose.pose.covariance[6] = Sigma_(1,0);
    // y-y (7,1)
    output_pose.pose.covariance[7] = Sigma_(1,1);
    // y-yaw (11,5) in ROS, corresponds to y-theta (1,2) in our state
    output_pose.pose.covariance[11] = Sigma_(1,2);

    // yaw-x (30,0) in ROS, corresponds to theta-x (2,0) in our state
    output_pose.pose.covariance[30] = Sigma_(2,0);
    // yaw-y (31,1) in ROS, corresponds to theta-y (2,1) in our state
    output_pose.pose.covariance[31] = Sigma_(2,1);
    // yaw-yaw (35,5) in ROS, corresponds to theta-theta (2,2) in our state
    output_pose.pose.covariance[35] = Sigma_(2,2);

    // All other covariances are 0, as we are modeling a 2D robot and not explicitly
    // providing uncertainty for z, roll, or pitch, nor for cross-covariances involving them.

    pub_->publish(output_pose);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}
