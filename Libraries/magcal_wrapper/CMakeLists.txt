cmake_minimum_required(VERSION 3.10)
project(magcal_wrapper)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(oscp_imu_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MagnetometerCalibration.action"
)

ament_python_install_package(magcal_wrapper_py)

install(PROGRAMS magcal_wrapper_py/magnetometer_calibration.py
  DESTINATION lib/${PROJECT_NAME} RENAME magnetometer_calibration)
install(PROGRAMS magcal_wrapper_py/mag_calibration_client.py
  DESTINATION lib/${PROJECT_NAME} RENAME mag_calibration_client)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
