cmake_minimum_required(VERSION 3.5)
project(avder)

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
find_package(rosbag2_cpp REQUIRED)
find_package(v2x_msg REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ${v2x_msg_INCLUDE_DIRS}
)

add_executable(edr src/edr.cpp)
ament_target_dependencies(edr rclcpp std_msgs rosbag2_cpp sensor_msgs v2x_msg)

install(TARGETS
  edr
  DESTINATION lib/${PROJECT_NAME})

ament_package()