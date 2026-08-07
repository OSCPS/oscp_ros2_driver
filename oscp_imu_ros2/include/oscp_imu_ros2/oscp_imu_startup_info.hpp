#ifndef OSCP_IMU_ROS2_STARTUP_INFO_HPP
#define OSCP_IMU_ROS2_STARTUP_INFO_HPP

/* Includes */
#include <string>
#include <cstdint>

namespace oscp_imu {

struct StartupInfo {
    std::string mark_number;
    uint32_t unit_number;

    uint8_t operating_mode;

    uint8_t gyro_range;
    uint8_t accel_range;
    uint8_t incl_range;

    uint8_t gyro_filters;
    uint8_t gyro_lpf;
    uint8_t gyro_hpf;

    uint8_t accel_filters;
    uint8_t accel_lpf;
    uint8_t accel_hpf;
};

} // namespace oscp_imu

#endif // OSCP_IMU_ROS2_STARTUP_INFO_HPP