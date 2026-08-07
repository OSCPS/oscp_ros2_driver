cmake_minimum_required(VERSION 3.8)
project(oscp_imu_ros2)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Threads REQUIRED)
find_package(oscp_msgs REQUIRED)
find_package(oscp_libraries REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(motion_cal_wrapper REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Services
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/ZeroOrientation.srv"
    "srv/StationaryCalibrate.srv"
    "srv/GetIMUConfig.srv"
      
  DEPENDENCIES
    builtin_interfaces
    action_msgs
    std_msgs
    unique_identifier_msgs
)

# Node
add_executable(oscp_imu_node 
src/oscp_imu_node.cpp
src/mag_calibration_helper.cpp
)

add_executable(counter_tracker 
src/counter_tracker.cpp
src/mag_calibration_helper.cpp
)


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(oscp_imu_node
  "${cpp_typesupport_target}"
)

ament_export_dependencies(rosidl_default_runtime)

target_include_directories(oscp_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../Libraries/oscp_libraries/include
)

target_include_directories(counter_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../Libraries/oscp_libraries/include
)

ament_target_dependencies(oscp_imu_node
  rclcpp
  sensor_msgs
  std_msgs
  oscp_msgs
  rclcpp_action
  tf2
  motion_cal_wrapper
)

ament_target_dependencies(counter_tracker
  rclcpp
  sensor_msgs
  std_msgs
  oscp_msgs
  rclcpp_action
  tf2
  motion_cal_wrapper
)

target_link_libraries(oscp_imu_node
  oscp_libraries::oscp_libraries
  Threads::Threads
)

target_link_libraries(counter_tracker
  oscp_libraries::oscp_libraries
  Threads::Threads
)

install(TARGETS 
  oscp_imu_node
  counter_tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()