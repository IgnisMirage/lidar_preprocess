cmake_minimum_required(VERSION 3.8)
project(lidar_preprocess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(lidar_preprocess_node src/lidar_preprocess_node.cpp)
ament_target_dependencies(lidar_preprocess_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)
target_link_libraries(lidar_preprocess_node ${PCL_LIBRARIES})

install(TARGETS lidar_preprocess_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
