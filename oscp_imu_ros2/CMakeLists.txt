cmake_minimum_required(VERSION 3.8)
project(oscp_imu_ros2)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(oscp_msgs REQUIRED)
find_package(oscp_libraries REQUIRED)

# Node
add_executable(oscp_imu_node 
src/oscp_imu_node.cpp
)

target_include_directories(oscp_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(oscp_imu_node
  rclcpp
  sensor_msgs
  std_msgs
  oscp_msgs
)

target_link_libraries(oscp_imu_node
  oscp_libraries::oscp_libraries
  Threads::Threads
)

install(TARGETS oscp_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()