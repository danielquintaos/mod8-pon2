cmake_minimum_required(VERSION 3.8)
project(turtlebot3_nav_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_recoveries REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_voxel_grid REQUIRED)
find_package(navigation2 REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(turtlebot3 REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
find_package(turtlebot3_navigation REQUIRED)
find_package(turtlebot3_simulations REQUIRED)

install(DIRECTORY launch config scripts
  DESTINATION share/$mod8-pon2/
)

ament_package()

