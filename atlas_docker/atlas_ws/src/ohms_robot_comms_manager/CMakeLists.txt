cmake_minimum_required(VERSION 3.5)
project(ohms_robot_comms_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(ohms_robot_comms_subscriber src/ohms_robot_comms_subscriber.cpp)
ament_target_dependencies(ohms_robot_comms_subscriber rclcpp std_msgs nav_msgs octomap_msgs)

add_executable(ohms_robot_comms_publisher src/ohms_robot_comms_publisher.cpp)
ament_target_dependencies(ohms_robot_comms_publisher rclcpp std_msgs nav_msgs octomap_msgs)

install(TARGETS
  ohms_robot_comms_subscriber
  ohms_robot_comms_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()