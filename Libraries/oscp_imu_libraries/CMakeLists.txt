cmake_minimum_required(VERSION 3.8)
project(oscp_imu_libraries)

find_package(ament_cmake REQUIRED)

add_library(oscp_imu_libraries
  src/oscp_imu.c
  src/cobs.c
)

target_include_directories(oscp_imu_libraries PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS oscp_imu_libraries
  EXPORT export_oscp_imu_libraries
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_oscp_imu_libraries HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()