# TurtleBot3 Localization with KF, EKF, and PF

This project implements and compares three fundamental state estimation algorithms for 2D robot localization — **Kalman Filter (KF)**, **Extended Kalman Filter (EKF)**, and **Particle Filter (PF)** — using ROS 2 and Gazebo. The robot is navigated through predefined waypoints, and localization accuracy is visualized in RViz.

## Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git
cd YOUR_REPO_NAME
```

### 2. Build the Docker Image and Start the Container

```bash
docker build -t turtlebot3_localization .
docker compose up
```

### 3. Build the ROS 2 Workspace

Inside the container:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### 4. Launch the Simulation

```bash
ros2 launch turtlebot3_full_bringup full_bringup.launch.py
```

This will:

* Start **Gazebo** with a custom map and TurtleBot3 robot
* Launch **RViz**
* Run all three localization filters (KF, EKF, PF)
* Start automatic waypoint navigation

## Visualization Overview

* **Green Arrow** → Ground Truth Pose from Gazebo
* **Red Arrow** → Pose from EKF
* **Light Blue Arrow** → Pose from KF
* **PF** → Displayed as a `PoseArray`; the robot’s location in RViz reflects the **mean of the particle cloud**

## Results

RMS error of each filter (vs. ground truth in kidnapped scenario):

* **Kalman Filter (KF)**: `17.46 m`
* **Extended Kalman Filter (EKF)**: `3.11 m`
* **Particle Filter (PF)**: `0.29 m`

See `rms_errors_kidnapped.txt` for details.

## Filters Implementation

Each filter is implemented in C++ with:

* A `predict(...)` and `correct(...)` method
* ROS 2 nodes using sensor data (`/odom`, `/imu`)
* Output published to `/prediction` as `PoseWithCovarianceStamped`

All nodes operate on control input (velocities) — no absolute pose input is used.

## References

* [TurtleBot3 Simulation Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
* [ROS 2 Tutorials](http://wiki.ros.org/ROS/Tutorials)
* [Thrun, Burgard, Fox - Probabilistic Robotics (2005)](https://www.probabilistic-robotics.org/)
