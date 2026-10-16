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

inline std::string startupToOperatingMode(uint8_t value) {
    switch (value) {
        case 0: return "IDLE";
        case 1: return "LOW";
        case 2: return "MEDIUM";
        default:
            throw std::runtime_error("Invalid startup operating mode: " + std::to_string(value));
    }
}

inline std::string startupToGyroRange(uint8_t value) {
    switch (value) {
        case 2:  return "DPS_125";
        case 0:  return "DPS_250";
        case 4:  return "DPS_500";
        case 8:  return "DPS_1000";
        case 12: return "DPS_2000";
        case 1:  return "DPS_4000";
        default:
            throw std::runtime_error("Invalid startup gyro range: " + std::to_string(value));
    }
}

inline std::string startupToAccelRange(uint8_t value) {
    switch (value) {
        case 0: return "G_2";
        case 2: return "G_4";
        case 3: return "G_8";
        case 1: return "G_16";
        default:
            throw std::runtime_error("Invalid startup accel range: " + std::to_string(value));
    }
}

inline std::string startupToInclRange(uint8_t value) {
    switch (value) {
        case 0: return "G_0_5";
        case 2: return "G_1_0";
        case 3: return "G_2_0";
        case 1: return "G_3_0";
        default:
            throw std::runtime_error("Invalid startup inclinometer range: " + std::to_string(value));
    }
}

inline std::string startupToFilterMode(uint8_t value) {
    switch (value) {
        case 0: return "DISABLED";
        case 1: return "LP_ONLY";
        case 2: return "HP_ONLY";
        case 3: return "LP_AND_HP";
        default:
            throw std::runtime_error("Invalid startup filter mode: " + std::to_string(value));
    }
}

inline std::string startupToFilterCutoff(uint8_t value) {
    switch (value) {
        case 0: return "C0";
        case 1: return "C1";
        case 2: return "C2";
        case 3: return "C3";
        case 4: return "C4";
        case 5: return "C5";
        case 6: return "C6";
        case 7: return "C7";
        default:
            throw std::runtime_error("Invalid startup filter cutoff: " + std::to_string(value));
    }
}


} // namespace oscp_imu

#endif // OSCP_IMU_ROS2_STARTUP_INFO_HPP