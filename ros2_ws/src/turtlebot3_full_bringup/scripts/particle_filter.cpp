#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h" // Needed for tf2::Matrix3x3
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/empty.hpp" // Required for global localization service

#include <random>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>
#include <numeric>

// Define a simple struct for a particle
struct Particle {
    double x, y, theta; // Pose of the particle
    double weight;      // Weight of the particle
};

class ParticleFilterLocalization : public rclcpp::Node
{
public:
    ParticleFilterLocalization()
        : Node("particle_filter_localization"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          tf_broadcaster_(this)
    {
        num_particles_ = 1000;
        map_frame_ = "map";
        odom_frame_ = "odom";
        base_link_frame_ = "base_link";
        initial_x_ = 0.0;
        initial_y_ = 0.0;
        initial_theta_ = 0.0;
        odom_trans_noise_ = 0.05;
        odom_rot_noise_ = 0.01;
        laser_hit_std_dev_ = 0.2;
        laser_miss_likelihood_ = 0.01;
        neff_resample_threshold_ratio_ = 0.5;

        // Initialize particles. `false` means initial localized spread.
        initializeParticles(false);

        // Subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ParticleFilterLocalization::laserScanCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&ParticleFilterLocalization::odometryCallback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&ParticleFilterLocalization::mapCallback, this, std::placeholders::_1));

        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("localized_pose", 10);
        particle_cloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud_test", 10);

        // Service for global localization
        global_localization_service_ = this->create_service<std_srvs::srv::Empty>(
            "global_localization",
            std::bind(&ParticleFilterLocalization::globalLocalizationService, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Timer for publishing particle cloud (optional, for visualization)
        particle_cloud_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ParticleFilterLocalization::publishParticleCloud, this));

        RCLCPP_INFO(this->get_logger(), "Particle Filter Localization Node initialized. Waiting for map...");
    }

private:
    int num_particles_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_link_frame_;
    double initial_x_, initial_y_, initial_theta_;
    double odom_trans_noise_;
    double odom_rot_noise_;
    double laser_hit_std_dev_;
    double laser_miss_likelihood_;
    double neff_resample_threshold_ratio_;

    std::vector<Particle> particles_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_; // Stored map

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_cloud_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_localization_service_;
    rclcpp::TimerBase::SharedPtr particle_cloud_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::default_random_engine random_engine_; // Random number generator

    // --- Particle Filter Core Functions ---
    void initializeParticles(bool global_reinit)
    {
        particles_.resize(num_particles_);
        std::random_device rd;
        random_engine_.seed(rd());

        if (global_reinit && map_msg_) {
            RCLCPP_INFO(this->get_logger(), "Globally re-initializing particles across the map.");
            double map_min_x = map_msg_->info.origin.position.x;
            double map_min_y = map_msg_->info.origin.position.y;
            double map_max_x = map_min_x + map_msg_->info.width * map_msg_->info.resolution;
            double map_max_y = map_min_y + map_msg_->info.height * map_msg_->info.resolution;

            std::uniform_real_distribution<double> x_dist(map_min_x, map_max_x);
            std::uniform_real_distribution<double> y_dist(map_min_y, map_max_y);
            std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI); // Full 360 degrees

            for (int i = 0; i < num_particles_; ++i) {
                particles_[i].x = x_dist(random_engine_);
                particles_[i].y = y_dist(random_engine_);
                particles_[i].theta = theta_dist(random_engine_);
                particles_[i].weight = 1.0 / num_particles_;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Initializing particles normally around (%.2f, %.2f, %.2f).", initial_x_, initial_y_, initial_theta_);
            std::normal_distribution<double> x_dist(initial_x_, 0.5);
            std::normal_distribution<double> y_dist(initial_y_, 0.5);
            std::normal_distribution<double> theta_dist(initial_theta_, 0.2);

            for (int i = 0; i < num_particles_; ++i) {
                particles_[i].x = x_dist(random_engine_);
                particles_[i].y = y_dist(random_engine_);
                particles_[i].theta = theta_dist(random_engine_);
                particles_[i].weight = 1.0 / num_particles_;
            }
        }
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!last_odom_msg_) {
            last_odom_msg_ = msg;
            return;
        }
        // Calculate the relative motion between the last and current odometry readings
        // This motion is expressed in the robot's base_link frame (local frame).
        tf2::Transform T_odom_to_base_last;
        tf2::Transform T_odom_to_base_current;

        tf2::fromMsg(last_odom_msg_->pose.pose, T_odom_to_base_last);
        tf2::fromMsg(msg->pose.pose, T_odom_to_base_current);

        // T_relative_motion represents the motion of the robot's base_link from its previous pose
        // to its current pose, expressed in the *previous base_link frame*.
        tf2::Transform T_relative_motion = T_odom_to_base_last.inverse() * T_odom_to_base_current;

        double delta_x_local = T_relative_motion.getOrigin().x();
        double delta_y_local = T_relative_motion.getOrigin().y();

        // Corrected: Extract yaw from the relative motion quaternion
        tf2::Matrix3x3 m_relative(T_relative_motion.getRotation());
        double roll_relative, pitch_relative, delta_theta_local;
        m_relative.getRPY(roll_relative, pitch_relative, delta_theta_local);


        // Add noise to these local deltas
        std::normal_distribution<double> trans_noise_x(0.0, odom_trans_noise_);
        std::normal_distribution<double> trans_noise_y(0.0, odom_trans_noise_);
        std::normal_distribution<double> rot_noise(0.0, odom_rot_noise_);

        for (auto& p : particles_) {
            double noisy_delta_x_local = delta_x_local + trans_noise_x(random_engine_);
            double noisy_delta_y_local = delta_y_local + trans_noise_y(random_engine_);
            double noisy_delta_theta_local = delta_theta_local + rot_noise(random_engine_);

            p.x += noisy_delta_x_local * std::cos(p.theta) - noisy_delta_y_local * std::sin(p.theta);
            p.y += noisy_delta_x_local * std::sin(p.theta) + noisy_delta_y_local * std::cos(p.theta);
            p.theta += noisy_delta_theta_local;
            p.theta = fmod(p.theta + M_PI, 2 * M_PI) - M_PI; // Normalize angle to [-PI, PI]
        }

        last_odom_msg_ = msg;
    }


    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_msg_ = msg;
        RCLCPP_INFO(this->get_logger(), "Map received! Resolution: %.3f m, Width: %d, Height: %d",
                    map_msg_->info.resolution, map_msg_->info.width, map_msg_->info.height);
    }

    int getMapOccupancy(double x, double y)
    {
        if (!map_msg_) {
            return -1; // Map not loaded
        }

        // Convert map coordinates to grid cell coordinates
        double map_origin_x = map_msg_->info.origin.position.x;
        double map_origin_y = map_msg_->info.origin.position.y;
        double resolution = map_msg_->info.resolution;
        unsigned int map_width = map_msg_->info.width;
        unsigned int map_height = map_msg_->info.height;

        int grid_x = static_cast<int>((x - map_origin_x) / resolution);
        int grid_y = static_cast<int>((y - map_origin_y) / resolution);

        // Check if coordinates are within map bounds
        if (grid_x >= 0 && grid_x < static_cast<int>(map_width) &&
            grid_y >= 0 && grid_y < static_cast<int>(map_height))
        {
            return map_msg_->data[grid_y * map_width + grid_x];
        }
        return -1; // Out of bounds or unknown
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!map_msg_) {
            RCLCPP_WARN(this->get_logger(), "Map not received yet. Skipping laser scan update.");
            return;
        }
        if (particles_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Particles not initialized. Skipping laser scan update.");
            return;
        }

        // Get transform from base_link to laser_frame
        geometry_msgs::msg::TransformStamped base_to_laser_tf;
        try {
            base_to_laser_tf = tf_buffer_.lookupTransform(
                base_link_frame_, msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                          base_link_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
            return;
        }

        double total_weight = 0.0;
        std::normal_distribution<double> hit_noise(0.0, laser_hit_std_dev_);

        for (auto& p : particles_) {
            double particle_likelihood = 1.0; // Initialize likelihood for this particle

            // Transform from map to particle's base_link frame
            tf2::Transform map_to_particle_base_tf;
            map_to_particle_base_tf.setOrigin(tf2::Vector3(p.x, p.y, 0.0));
            map_to_particle_base_tf.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1), p.theta));

            // Combine transforms: map -> particle_base -> laser_frame
            // This is the transform from the map frame to the laser frame, assuming the particle's pose
            tf2::Transform map_to_particle_laser_tf = map_to_particle_base_tf * tf2::Transform(
                tf2::Quaternion(base_to_laser_tf.transform.rotation.x,
                                base_to_laser_tf.transform.rotation.y,
                                base_to_laser_tf.transform.rotation.z,
                                base_to_laser_tf.transform.rotation.w),

                tf2::Vector3(base_to_laser_tf.transform.translation.x,
                            base_to_laser_tf.transform.translation.y,
                            base_to_laser_tf.transform.translation.z));


            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                double range = msg->ranges[i];
                double angle = msg->angle_min + i * msg->angle_increment;

                // Skip invalid laser readings
                if (std::isinf(range) || std::isnan(range) || range < msg->range_min || range > msg->range_max) {
                    continue;
                }

                // Calculate the endpoint of the laser ray in the laser frame
                tf2::Vector3 laser_endpoint_laser_frame(
                    range * std::cos(angle),
                    range * std::sin(angle),
                    0.0
                );

                // Transform the laser endpoint from laser frame to map frame using the particle's pose
                tf2::Vector3 laser_endpoint_map_frame = map_to_particle_laser_tf * laser_endpoint_laser_frame;

                // Get occupancy value at the laser endpoint in the map
                int occupancy_value = getMapOccupancy(laser_endpoint_map_frame.x(), laser_endpoint_map_frame.y());

                // --- Measurement Model: Evaluate likelihood based on map occupancy ---
                if (occupancy_value == 100) { // Laser ray ends in an occupied cell (good match)
                    // High likelihood for hitting an obstacle
                    particle_likelihood *= std::exp(-0.5 * std::pow(hit_noise(random_engine_) / laser_hit_std_dev_, 2));
                } else if (occupancy_value == 0) { // Laser ray ends in a free cell (bad match, unless max range)
                    // Low likelihood if a definite hit occurs in free space
                    if (range < msg->range_max - 0.1) { // If it's a definite hit, but in free space
                        particle_likelihood *= laser_miss_likelihood_; // Very low likelihood
                    } else { // If it's max range, it's consistent with free space
                        particle_likelihood *= 1.0; // Neutral likelihood
                    }
                } else { // -1 (unknown) or other values (neutral evidence)
                    particle_likelihood *= 0.5; // Neutral likelihood for unknown or out of bounds
                }
            }
            p.weight *= particle_likelihood; // Multiply by likelihood to update weight
            total_weight += p.weight;
        }

        // Normalize weights
        if (total_weight > 0) {
            for (auto& p : particles_) {
                p.weight /= total_weight;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Total weight is zero after measurement update. Re-initializing particles globally.");
            initializeParticles(true); // Global re-initialization if all weights are zero
            return;
        }

        // Calculate Effective Number of Particles (Neff)
        double sum_sq_weights = 0.0;
        for (const auto& p : particles_) {
            sum_sq_weights += std::pow(p.weight, 2);
        }
        double neff = (sum_sq_weights > 0) ? 1.0 / sum_sq_weights : 0.0;

        // Resample particles if Neff is below threshold. This is the "selection" step.
        if (neff < neff_resample_threshold_ratio_ * num_particles_) {
            RCLCPP_INFO(this->get_logger(), "Neff (%.2f) below threshold (%.2f). Resampling particles.", neff, neff_resample_threshold_ratio_ * num_particles_);
            resampleParticles();
        }


        // Estimate and publish robot pose
        publishRobotPose(msg->header.stamp);
    }

    void resampleParticles()
    {
        std::vector<Particle> new_particles(num_particles_);
        std::vector<double> cumulative_weights(num_particles_);

        // Calculate cumulative weights
        cumulative_weights[0] = particles_[0].weight;
        for (int i = 1; i < num_particles_; ++i) {
            cumulative_weights[i] = cumulative_weights[i-1] + particles_[i].weight;
        }

        // Low Variance Resampling (Roulette Wheel variant)
        std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
        double r = dist(random_engine_); // Initial random number

        int index = 0;
        for (int i = 0; i < num_particles_; ++i) {
            double target_weight = r + (double)i / num_particles_; // Target for current particle
            while (index < num_particles_ - 1 && target_weight > cumulative_weights[index]) {
                index++;
            }
            new_particles[i] = particles_[index];
            new_particles[i].weight = 1.0 / num_particles_; // Reset weights to equal after resampling
        }
        particles_ = new_particles;
    }

    void publishRobotPose(const rclcpp::Time& stamp)
    {
        // Estimate robot pose as the weighted average of particles
        double estimated_x = 0.0;
        double estimated_y = 0.0;
        double sin_avg = 0.0;
        double cos_avg = 0.0;

        for (const auto& p : particles_) {
            estimated_x += p.x * p.weight;
            estimated_y += p.y * p.weight;
            sin_avg += std::sin(p.theta) * p.weight;
            cos_avg += std::cos(p.theta) * p.weight;
        }

        double estimated_theta = std::atan2(sin_avg, cos_avg);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = map_frame_;
        pose_msg.pose.position.x = estimated_x;
        pose_msg.pose.position.y = estimated_y;
        pose_msg.pose.position.z = 0.0; // Assuming 2D localization

        tf2::Quaternion q;
        q.setRPY(0, 0, estimated_theta);
        pose_msg.pose.orientation = tf2::toMsg(q);

        pose_pub_->publish(pose_msg);

        // Publish map -> odom transform
        geometry_msgs::msg::TransformStamped map_to_odom_tf;
        map_to_odom_tf.header.stamp = stamp;
        map_to_odom_tf.header.frame_id = map_frame_;
        map_to_odom_tf.child_frame_id = odom_frame_;

        // Get the current transform from odom to base_link
        geometry_msgs::msg::TransformStamped odom_to_base_tf;
        try {
            odom_to_base_tf = tf_buffer_.lookupTransform(
                odom_frame_, base_link_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not look up transform from %s to %s: %s",
                          odom_frame_.c_str(), base_link_frame_.c_str(), ex.what());
            return;
        }

        // Calculate the transform from map to base_link (estimated pose)
        tf2::Transform map_to_base_tf;
        map_to_base_tf.setOrigin(tf2::Vector3(estimated_x, estimated_y, 0.0));
        map_to_base_tf.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1), estimated_theta));

        // map_to_odom = map_to_base * (odom_to_base)^-1
        tf2::Transform odom_to_base_tf2;
        tf2::fromMsg(odom_to_base_tf.transform, odom_to_base_tf2);
        tf2::Transform map_to_odom_tf2 = map_to_base_tf * odom_to_base_tf2.inverse();

        map_to_odom_tf.transform = tf2::toMsg(map_to_odom_tf2);
        tf_broadcaster_.sendTransform(map_to_odom_tf);
    }

    void publishParticleCloud()
    {
        geometry_msgs::msg::PoseArray particle_cloud_msg;
        particle_cloud_msg.header.stamp = this->now();
        particle_cloud_msg.header.frame_id = map_frame_;

        for (const auto& p : particles_) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p.theta);
            pose.orientation = tf2::toMsg(q);
            particle_cloud_msg.poses.push_back(pose);
        }
        particle_cloud_pub_->publish(particle_cloud_msg);
    }

    void globalLocalizationService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request; // Suppress unused parameter warning
        (void)response; // Suppress unused parameter warning
        if (map_msg_) {
            initializeParticles(true); // Trigger global re-initialization
            RCLCPP_INFO(this->get_logger(), "Global localization service called: Particles re-initialized across map.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Global localization service called, but map not loaded. Cannot re-initialize globally.");
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterLocalization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
