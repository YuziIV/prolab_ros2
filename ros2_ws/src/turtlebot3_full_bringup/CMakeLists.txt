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
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# === BUILD FILTER NODE ===
add_executable(extended_kalman_filter scripts/extended_kalman_filter.cpp)
ament_target_dependencies(extended_kalman_filter
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  rclcpp
  gazebo_msgs
  geometry_msgs
)
add_executable(kalman_filter scripts/kalman_filter.cpp)
ament_target_dependencies(kalman_filter
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  rclcpp
  gazebo_msgs
  geometry_msgs
)
add_executable(ground_truth scripts/ground_truth.cpp)
ament_target_dependencies(ground_truth
  rclcpp
  gazebo_msgs
  geometry_msgs
)

add_executable(particle_filter scripts/particle_filter.cpp)
ament_target_dependencies(particle_filter
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  rclcpp
  gazebo_msgs
  geometry_msgs
)

add_executable(nav2_goals scripts/nav2_goals.cpp)
ament_target_dependencies(nav2_goals
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(nav2_goals yaml-cpp)

install(TARGETS
  particle_filter
  ground_truth
  kalman_filter
  extended_kalman_filter
  nav2_goals
  DESTINATION lib/${PROJECT_NAME}
)

# === INSTALL NON-C++ ASSETS ===
install(DIRECTORY
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
