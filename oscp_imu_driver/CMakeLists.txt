cmake_minimum_required(VERSION 3.10)
project(oscp_imu_driver)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(oscp_imu_msgs REQUIRED)
find_package(oscp_imu_libraries REQUIRED)
find_package(magcal_wrapper REQUIRED)

# Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ZeroOrientation.srv"
  "srv/StationaryCalibrate.srv"
  "srv/GetIMUConfig.srv"
  "srv/ApplyMagConfig.srv"
  DEPENDENCIES
    builtin_interfaces
    action_msgs
    std_msgs
    unique_identifier_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Nodes
add_executable(oscp_imu_node src/oscp_imu_node.cpp)
add_executable(counter_tracker src/counter_tracker.cpp)

foreach(target oscp_imu_node counter_tracker)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${target}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    tf2::tf2
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${oscp_imu_msgs_TARGETS}
    ${magcal_wrapper_TARGETS}
    oscp_imu_libraries::oscp_imu_libraries
    Threads::Threads
  )
endforeach()

target_link_libraries(oscp_imu_node
  "${cpp_typesupport_target}"
  yaml-cpp::yaml-cpp
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
