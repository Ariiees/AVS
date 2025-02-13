cmake_minimum_required(VERSION 3.10)
project(computation)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(computation pointcloud_filter.cpp)
target_link_libraries(computation ${PCL_LIBRARIES})

add_executable(count count.cpp)
target_link_libraries(count ${PCL_LIBRARIES})