cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)  # Enable modern policy for find_package
project(pcl_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add executable
add_executable(pcl_filter_node src/pcl_filter_node.cpp)

# Link against required libraries
target_link_libraries(pcl_filter_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(pcl_filter_node
  rclcpp
  sensor_msgs
  pcl_ros
  PCL
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS pcl_filter_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
