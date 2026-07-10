cmake_minimum_required(VERSION 3.8)
project(oscp_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OscpRaw.msg"
  "msg/OscpRot.msg"
  "msg/OscpQuat.msg"
  "msg/OscpEuler.msg"
  "msg/OscpGnss.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Runtime export
ament_export_dependencies(rosidl_default_runtime)

ament_package()