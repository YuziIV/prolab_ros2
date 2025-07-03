#include "extended_kalman_filter.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For converting Quaternion to RPY

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
    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/EKF_pose", 10);

    // Initialize state and covariance
    mu_ = Eigen::VectorXd::Zero(6);
    Sigma_ = Eigen::MatrixXd::Identity(6, 6) * 1e-9; // Small initial covariance

    // Initialize process noise covariance (Q)
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    Q_(0, 0) = 0.05; // x
    Q_(1, 1) = 0.05; // y
    Q_(2, 2) = 0.02; // theta
    Q_(3, 3) = 0.1;  // vx
    Q_(4, 4) = 0.1;  // vy
    Q_(5, 5) = 0.05; // wz

    // Initialize measurement noise covariance (R_imu) for IMU measurements
    R_imu_ = Eigen::MatrixXd::Identity(3, 3);
    R_imu_(0, 0) = 0.01; // wz (gyro_z) noise
    R_imu_(1, 1) = 0.1;  // ax (accel_x) noise
    R_imu_(2, 2) = 0.1;  // ay (accel_y) noise

    last_time_ = this->now();
    first_measurement_received_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Extened_Kalman Filter Node Initialized!");
}

// Prediction Step (uses Odometry as motion model)
void FilterNode::predict(double dt, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    // Extract linear and angular velocities from odometry
    double linear_x_odom = odom_msg->twist.twist.linear.x;
    double linear_y_odom = odom_msg->twist.twist.linear.y;
    double angular_z_odom = odom_msg->twist.twist.angular.z;

    // Current state variables
    double x = mu_(0);
    double y = mu_(1);
    double theta = mu_(2);
    // double vx = mu_(3); // We use odom velocities as input, not from the state directly
    // double vy = mu_(4);
    // double wz = mu_(5);

    // --- Non-linear motion model (f(mu, u)) ---
    // Update position and orientation using velocities and dt
    // For differential drive, typically only linear_x_odom and angular_z_odom are significant.
    // If linear_y_odom is non-zero, it means side slip or a holonomic robot.
    double delta_x = (linear_x_odom * std::cos(theta) - linear_y_odom * std::sin(theta)) * dt;
    double delta_y = (linear_x_odom * std::sin(theta) + linear_y_odom * std::cos(theta)) * dt;
    double delta_theta = angular_z_odom * dt;

    mu_ (0) += delta_x;
    mu_ (1) += delta_y;
    mu_ (2) += delta_theta;
    mu_ (3) = linear_x_odom; // Set state vx to current odom vx
    mu_ (4) = linear_y_odom; // Set state vy to current odom vy
    mu_ (5) = angular_z_odom; // Set state wz to current odom wz

    // Normalize theta to [-PI, PI]
    mu_(2) = std::fmod(mu_(2), 2 * M_PI);
    if (mu_(2) > M_PI) mu_(2) -= 2 * M_PI;
    if (mu_(2) < -M_PI) mu_(2) += 2 * M_PI;

    // --- Compute Jacobian of the motion model (F_t = A_t in your algorithm) ---
    // F_t = df/d(mu)
    // For x: x_k+1 = x_k + (vx_k*cos(theta_k) - vy_k*sin(theta_k)) * dt
    // For y: y_k+1 = y_k + (vx_k*sin(theta_k) + vy_k*cos(theta_k)) * dt
    // For theta: theta_k+1 = theta_k + wz_k * dt
    // For vx, vy, wz: We assume they become the new odom values directly, or a simple persistence model.
    // Here, we simplify by assuming vx, vy, wz are effectively direct inputs,
    // so their rows in F will be mostly zeros or ones.

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 2) = (-linear_x_odom * std::sin(theta) - linear_y_odom * std::cos(theta)) * dt; // dx/d(theta)
    F(1, 2) = (linear_x_odom * std::cos(theta) - linear_y_odom * std::sin(theta)) * dt;  // dy/d(theta)

    // For simplicity, we are assuming vx, vy, wz in the state are updated by the odom readings
    // so their influence on the prediction of themselves is '1' and on others '0'.
    // Their relation to x, y, theta is handled by the non-linear update above.

    // --- Prediction of Covariance (Sigma_t = F * Sigma_t-1 * F^T + Q) ---
    Sigma_ = F * Sigma_ * F.transpose() + Q_;

    RCLCPP_INFO(this->get_logger(), "Prediction: x=%.2f, y=%.2f, theta=%.2f", mu_(0), mu_(1), mu_(2));
}

// Correction Step (uses IMU data as measurement)
void FilterNode::correct(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
    // --- Extract IMU measurements (z_t) ---
    // We are interested in angular velocity around Z (gyro_z), linear acceleration X and Y
    // IMU linear acceleration is usually in the IMU's body frame.
    // To use it for correction in the global frame, we need to rotate it by current estimated theta.

    double measured_wz = imu_msg->angular_velocity.z;
    double measured_ax_body = imu_msg->linear_acceleration.x;
    double measured_ay_body = imu_msg->linear_acceleration.y;

    // Rotate linear acceleration from IMU body frame to robot's base frame (which is aligned with odom frame in 2D)
    // We assume the IMU's frame is aligned with the robot's base_link frame.
    // If not, a static transform (TF) would be needed here.
    // For a 2D robot, current estimated yaw (mu_(2)) is enough for rotation.
    double estimated_theta = mu_(2);

    // Rotate acceleration from body frame to global frame (or odom frame)
    double measured_ax_global = measured_ax_body * std::cos(estimated_theta) - measured_ay_body * std::sin(estimated_theta);
    double measured_ay_global = measured_ax_body * std::sin(estimated_theta) + measured_ay_body * std::cos(estimated_theta);


    // Measurement vector z_t = [wz_imu, ax_imu_global, ay_imu_global]
    Eigen::VectorXd z = Eigen::VectorXd(3);
    z << measured_wz, measured_ax_global, measured_ay_global;

    Eigen::VectorXd z_wz_only = Eigen::VectorXd(1);
    z_wz_only << measured_wz;

    Eigen::MatrixXd H_wz_only = Eigen::MatrixXd::Zero(1, 6);
    H_wz_only(0, 5) = 1.0; // We measure wz directly

    Eigen::MatrixXd R_wz_only = Eigen::MatrixXd::Identity(1,1) * R_imu_(0,0); // Use the wz noise from R_imu_

    // --- Calculate Kalman Gain (K_t) ---
    // K_t = Sigma_bar * H_t^T * (H_t * Sigma_bar * H_t^T + R_t)^-1
    Eigen::MatrixXd S = H_wz_only * Sigma_ * H_wz_only.transpose() + R_wz_only;
    Eigen::MatrixXd K = Sigma_ * H_wz_only.transpose() * S.inverse();

    // --- Update State (mu_t) ---
    // mu_t = mu_bar_t + K_t * (z_t - H_t * mu_bar_t)
    mu_ = mu_ + K * (z_wz_only - H_wz_only * mu_);

    // --- Update Covariance (Sigma_t) ---
    // Sigma_t = (I - K_t * H_t) * Sigma_bar_t
    Sigma_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_wz_only) * Sigma_;

    // Normalize theta again after correction
    mu_(2) = std::fmod(mu_(2), 2 * M_PI);
    if (mu_(2) > M_PI) mu_(2) -= 2 * M_PI;
    if (mu_(2) < -M_PI) mu_(2) += 2 * M_PI;

    RCLCPP_INFO(this->get_logger(), "Correction: wz=%.2f, K(0)=%.2e, K(1)=%.2e, K(2)=%.2e", measured_wz, K(0,0), K(1,0), K(2,0));
}


// Callback for synchronized sensor data
void FilterNode::sensorCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                                const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
    if (!first_measurement_received_) {
        // Initialize state with first odometry reading
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
        RCLCPP_INFO(this->get_logger(), "Initialized Extended Kalman Filter state with first odometry.");
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

    predict(dt, odom_msg); // Pass odom_msg to prediction for its velocity data

    correct(imu_msg); // Pass imu_msg to correction

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

    // Fill covariance matrix (flattened for ROS message)
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            // Map our 6x6 state covariance to ROS 6x6 pose covariance
            // ROS covariance is x, y, z, roll, pitch, yaw
            // Our state is x, y, theta, vx, vy, wz
            // So we map (x,y,theta) from our state to (x,y,yaw) in ROS message
            // Fill diagonal for now, mapping based on index.
            if (i < 3 && j < 3) { // x, y, theta corresponds to x, y, yaw in ROS
                 output_pose.pose.covariance[6*i + j] = Sigma_(i, j);
            } else if (i == 2 && j == 5) { // yaw covariance with angular_velocity (wz)
                output_pose.pose.covariance[6*5 + 2] = Sigma_(2,5);
                output_pose.pose.covariance[6*2 + 5] = Sigma_(5,2);
            }
             else if (i == 5 && j == 5) { // wz covariance, does not directly map to pose cov
                // Not directly mapping velocity covariances to pose covariance for simplicity
                // For full covariance, you'd need to consider a 6x6 pose covariance for x,y,z,roll,pitch,yaw
                // and fill only relevant parts.
            }
        }
    }
    // A more precise mapping for 2D would be:
    output_pose.pose.covariance[0] = Sigma_(0,0);  // x-x
    output_pose.pose.covariance[1] = Sigma_(0,1);  // x-y
    output_pose.pose.covariance[5] = Sigma_(0,2);  // x-theta

    output_pose.pose.covariance[6] = Sigma_(1,0);  // y-x
    output_pose.pose.covariance[7] = Sigma_(1,1);  // y-y
    output_pose.pose.covariance[11] = Sigma_(1,2); // y-theta

    output_pose.pose.covariance[30] = Sigma_(2,0);  // theta-x
    output_pose.pose.covariance[31] = Sigma_(2,1);  // theta-y
    output_pose.pose.covariance[35] = Sigma_(2,2); // theta-theta

    // All other covariances are 0 for a 2D pose with no Z, Roll, Pitch uncertainty explicitly.
    // For a more complete covariance, you'd fill all 36 elements based on your 6x6 Sigma_
    // and potentially adding noise for unmeasured dimensions (z, roll, pitch).
    // For simplicity, we are setting others to zero or leaving them at default (which is usually zero).


    pub_->publish(output_pose);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}