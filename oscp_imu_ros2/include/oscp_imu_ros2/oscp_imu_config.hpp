#pragma once

#include <string>

namespace imu
{

enum class Transport { RS422, CANFD };

enum class OperatingMode { IDLE, LOW, MEDIUM };

enum class GyroRange { DPS_125, DPS_250, DPS_500, DPS_1000, DPS_2000, DPS_4000 };

enum class AccelRange { G_2, G_4, G_8, G_16 };

enum class InclRange { G_0_5, G_1_0, G_2_0, G_3_0 };

enum class FilterMode { DISABLED, LP_ONLY, HP_ONLY, LP_AND_HP };

enum class FilterCutoff { C0, C1, C2, C3, C4, C5, C6, C7 };

enum class AHRSConvention { NWU, ENU, NED };

enum class AHRSHeadingSource { NONE, INTERNAL_MAGNETOMETER };


struct IMUConfig
{
    std::string device = "/dev/ttyUSB0";
    int baudrate = 921600;
    Transport transport = Transport::RS422;

    bool enable_raw = false;
    bool enable_euler = false;
    bool enable_quat = true;
    bool enable_rot = false;
    bool enable_gnss = false;
    bool enable_ros = true;

    OperatingMode operating_mode = OperatingMode::MEDIUM;

    GyroRange gyro_range = GyroRange::DPS_500;
    AccelRange accel_range = AccelRange::G_4;
    InclRange incl_range = InclRange::G_1_0;

    FilterMode gyro_filter_mode = FilterMode::LP_ONLY;
    FilterMode accel_filter_mode = FilterMode::LP_ONLY;

    FilterCutoff gyro_lpf = FilterCutoff::C3;
    FilterCutoff gyro_hpf = FilterCutoff::C0;
    FilterCutoff accel_lpf = FilterCutoff::C3;
    FilterCutoff accel_hpf = FilterCutoff::C0;

    bool misalignment = false;

    AHRSConvention ahrs_convention = AHRSConvention::NWU;
    AHRSHeadingSource ahrs_heading = AHRSHeadingSource::NONE;
};

}