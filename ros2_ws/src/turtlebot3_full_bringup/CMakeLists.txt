cmake_minimum_required(VERSION 3.8)
project(turtlebot3_full_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(xacro REQUIRED)

# === BUILD FILTER NODE ===
add_executable(kalman_filter scripts/kalman_filter.cpp)
ament_target_dependencies(kalman_filter
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  Eigen3
)
add_executable(ground_truth scripts/ground_truth.cpp)
ament_target_dependencies(ground_truth
  rclcpp
  gazebo_msgs
  geometry_msgs
)

install(TARGETS
  kalman_filter
  ground_truth
  DESTINATION lib/${PROJECT_NAME}
)

# === INSTALL NON-C++ ASSETS ===
install(DIRECTORY
  assets
  launch
  maps
  rviz
  scripts
  test
  turtlebot3_full_bringup
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
include_directories(${EIGEN3_INCLUDE_DIRS})
ament_package()
