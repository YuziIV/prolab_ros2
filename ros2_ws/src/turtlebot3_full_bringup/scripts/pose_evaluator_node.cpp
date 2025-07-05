#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>

// Define a struct to hold synchronized pose data for logging and RMS calculation
struct PoseData {
    rclcpp::Time timestamp;
    geometry_msgs::msg::Pose gt_pose;
    geometry_msgs::msg::Pose kf_pose;
    geometry_msgs::msg::Pose ekf_pose;
    geometry_msgs::msg::Pose pf_pose;
};

// Define the SyncPolicy outside the class so it's accessible for member declaration
typedef message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::PoseStamped,               // Corresponds to gt_sub_
    geometry_msgs::msg::PoseWithCovarianceStamped, // Corresponds to kf_sub_
    geometry_msgs::msg::PoseWithCovarianceStamped, // Corresponds to ekf_sub_
    geometry_msgs::msg::PoseStamped                // Corresponds to pf_sub_
> SyncPolicy;

class PoseEvaluatorNode : public rclcpp::Node
{
public:
    PoseEvaluatorNode() : Node("pose_evaluator_node"),
                          tf_buffer_(this->get_clock()), // Keep TF buffer/listener if you need other transforms later
                          tf_listener_(tf_buffer_)
    {
        // Subscribers
        gt_sub_.subscribe(this, "/burger/ground_truth_pose");
        kf_sub_.subscribe(this, "/KF_pose");
        ekf_sub_.subscribe(this, "/EKF_pose");
        pf_sub_.subscribe(this, "/localized_pose"); // Assuming this is the topic for Particle Filter pose

        // Time Synchronizer Policy (ApproximateTime for robustness)
        // Order of subscribers here must match the SyncPolicy and callback arguments
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), gt_sub_, kf_sub_, ekf_sub_, pf_sub_);
        sync_->registerCallback(std::bind(&PoseEvaluatorNode::synchronizedCallback, this,
                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

        // Create output files
        paths_file_.open("paths.csv");
        if (paths_file_.is_open()) {
            paths_file_ << "timestamp,gt_x,gt_y,kf_x,kf_y,ekf_x,ekf_y,pf_x,pf_y\n";
            RCLCPP_INFO(this->get_logger(), "Opened paths.csv for writing.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open paths.csv");
        }

        rms_file_.open("rms_errors.txt");
        if (rms_file_.is_open()) {
            RCLCPP_INFO(this->get_logger(), "Opened rms_errors.txt for writing.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open rms_errors.txt");
        }

        RCLCPP_INFO(this->get_logger(), "Pose Evaluator Node has been started.");
    }

    ~PoseEvaluatorNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Pose Evaluator Node. Calculating RMS errors...");
        calculateAndLogRMSE();
        if (paths_file_.is_open()) {
            paths_file_.close();
        }
        if (rms_file_.is_open()) {
            rms_file_.close();
        }
    }

private:
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> gt_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> kf_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> ekf_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pf_sub_; // PF subscriber declaration

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // TF buffer and listener are no longer strictly needed for PF pose if subscribed directly,
    // but kept in case other TF functionalities are desired.
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<PoseData> synchronized_data_;
    std::ofstream paths_file_;
    std::ofstream rms_file_;

    // Corrected synchronizedCallback signature to match SyncPolicy and subscriber order
    void synchronizedCallback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& gt_msg,
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& kf_msg,
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& ekf_msg,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pf_msg) // Corrected order for pf_msg
    {
        PoseData current_data;
        current_data.timestamp = gt_msg->header.stamp; // Use ground truth timestamp as reference

        // Store poses
        current_data.gt_pose = gt_msg->pose;
        current_data.kf_pose = kf_msg->pose.pose;
        current_data.ekf_pose = ekf_msg->pose.pose;
        current_data.pf_pose = pf_msg->pose; // Directly use the subscribed PF pose

        synchronized_data_.push_back(current_data);

        // Write data to paths.csv immediately
        if (paths_file_.is_open()) {
            paths_file_ << std::fixed << std::setprecision(6)
                        << current_data.timestamp.seconds() << ","
                        << current_data.gt_pose.position.x << "," << current_data.gt_pose.position.y << ","
                        << current_data.kf_pose.position.x << "," << current_data.kf_pose.position.y << ","
                        << current_data.ekf_pose.position.x << "," << current_data.ekf_pose.position.y << ","
                        << current_data.pf_pose.position.x << "," << current_data.pf_pose.position.y << "\n";
        }
    }

    void calculateAndLogRMSE()
    {
        double kf_sq_error_sum = 0.0;
        double ekf_sq_error_sum = 0.0;
        double pf_sq_error_sum = 0.0;
        int kf_count = 0;
        int ekf_count = 0;
        int pf_count = 0;

        for (const auto& data : synchronized_data_) {
            double gt_x = data.gt_pose.position.x;
            double gt_y = data.gt_pose.position.y;

            // KF Error
            double kf_dx = data.kf_pose.position.x - gt_x;
            double kf_dy = data.kf_pose.position.y - gt_y;
            kf_sq_error_sum += (kf_dx * kf_dx) + (kf_dy * kf_dy);
            kf_count++;

            // EKF Error
            double ekf_dx = data.ekf_pose.position.x - gt_x;
            double ekf_dy = data.ekf_pose.position.y - gt_y;
            ekf_sq_error_sum += (ekf_dx * ekf_dx) + (ekf_dy * ekf_dy);
            ekf_count++;

            // PF Error (always available if synchronized)
            double pf_dx = data.pf_pose.position.x - gt_x;
            double pf_dy = data.pf_pose.position.y - gt_y;
            pf_sq_error_sum += (pf_dx * pf_dx) + (pf_dy * pf_dy);
            pf_count++;
        }

        double kf_rmse = (kf_count > 0) ? std::sqrt(kf_sq_error_sum / kf_count) : 0.0;
        double ekf_rmse = (ekf_count > 0) ? std::sqrt(ekf_sq_error_sum / ekf_count) : 0.0;
        double pf_rmse = (pf_count > 0) ? std::sqrt(pf_sq_error_sum / pf_count) : 0.0;

        if (rms_file_.is_open()) {
            rms_file_ << std::fixed << std::setprecision(6);
            rms_file_ << "RMS Error (KF vs GT): " << kf_rmse << " m\n";
            rms_file_ << "RMS Error (EKF vs GT): " << ekf_rmse << " m\n";
            rms_file_ << "RMS Error (PF vs GT): " << pf_rmse << " m\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "RMS file not open.");
        }

        RCLCPP_INFO(this->get_logger(), "--- RMS Results ---");
        RCLCPP_INFO(this->get_logger(), "KF RMSE: %.6f m", kf_rmse);
        RCLCPP_INFO(this->get_logger(), "EKF RMSE: %.6f m", ekf_rmse);
        RCLCPP_INFO(this->get_logger(), "PF RMSE: %.6f m", pf_rmse);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseEvaluatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}