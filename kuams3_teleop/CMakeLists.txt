cmake_minimum_required(VERSION 3.8)
project(kuams3_teleop)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the executable
add_executable(kuams3_teleop src/kuams3_teleop.cpp)
ament_target_dependencies(kuams3_teleop rclcpp geometry_msgs sensor_msgs)

# Install the executable
install(TARGETS kuams3_teleop
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
