cmake_minimum_required(VERSION 3.8)

project(motion_cal_wrapper)

# Standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


#################################################
# Dependencies
#################################################

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(oscp_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)


#################################################
# Interfaces
#################################################

rosidl_generate_interfaces(${PROJECT_NAME}

  "srv/ResetCalibration.srv"
  "msg/MagCalibration.msg"
  "action/MagnetometerCalibration.action"

  DEPENDENCIES
    std_msgs

)


#################################################
# C Calibration Library
#################################################

add_library(motioncal_lib

    src/magcal.c
    src/matrix.c
    src/quality.c
    src/imuread.c

)


target_include_directories(motioncal_lib PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/include

)


target_link_libraries(motioncal_lib

    m

)


#################################################
# Motion Calibration ROS2 Node
#################################################

add_executable(motion_cal_node
    src/motion_cal_node.cpp
    src/magcal.c
    src/matrix.c
    src/quality.c
    src/motioncal_shim.c
)


target_include_directories(motion_cal_node PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/include

)

target_include_directories(motion_cal_node PRIVATE
  include
  include/motion_cal_wrapper
)


ament_target_dependencies(motion_cal_node
  rclcpp
  oscp_msgs
  std_msgs
)


target_link_libraries(motion_cal_node

    motioncal_lib

)


#################################################
# Link ROS interfaces
#################################################

rosidl_get_typesupport_target(
    rosidl_typesupport_cpp_target
    ${PROJECT_NAME}
    rosidl_typesupport_cpp
)


target_link_libraries(motion_cal_node

    ${rosidl_typesupport_cpp_target}

)


#################################################
# Install
#################################################

install(

    TARGETS

        motion_cal_node
        motioncal_lib

    DESTINATION

        lib/${PROJECT_NAME}

)


install(

    DIRECTORY

        include/

    DESTINATION

        include

)



#################################################
# Export
#################################################

ament_export_include_directories(

    include

)


ament_export_dependencies(

    rclcpp
    rosidl_default_runtime
    action_msgs
    builtin_interfaces

)


ament_package()