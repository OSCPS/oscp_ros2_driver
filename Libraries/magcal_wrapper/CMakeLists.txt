cmake_minimum_required(VERSION 3.8)
project(magcal_wrapper)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(oscp_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MagnetometerCalibration.action"
)

install(
  DIRECTORY magcal_wrapper_py/
  DESTINATION lib/python3.10/site-packages/magcal_wrapper_py
)

install(
  PROGRAMS
    magcal_wrapper_py/magnetometer_calibration.py
    magcal_wrapper_py/mag_calibration_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()