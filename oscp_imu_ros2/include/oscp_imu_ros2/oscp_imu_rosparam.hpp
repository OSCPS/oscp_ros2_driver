#ifndef OSCP_IMU_ROS2_OSCP_IMU_ROSPARAM_HPP
#define OSCP_IMU_ROS2_OSCP_IMU_ROSPARAM_HPP

/* Includes */
#include "oscp_imu_ros2/oscp_imu_config.hpp"
#include "oscp_imu.h"

/* Translation functions */
namespace oscp_imu {

    inline oscp_transport_t to_oscp(Transport t) {
        switch(t) {
            case Transport::RS422:
                return OSCP_TRANSPORT_RS422;

            case Transport::CANFD:
                return OSCP_TRANSPORT_CANFD;

            default:
                throw std::runtime_error("Invalid transport");
        }
    }

    inline oscp_om_sel_t to_oscp(OperatingMode m) {
        switch(m) {
            case OperatingMode::IDLE:
                return OSCP_OM_IDLE;

            case OperatingMode::LOW:
                return OSCP_OM_LOW;

            case OperatingMode::MEDIUM:
                return OSCP_OM_MEDIUM;

            default:
                throw std::runtime_error("Invalid operating mode");
        }
    }

    inline oscp_gyro_dr_t to_oscp(GyroRange r) {
        switch(r) {
            case GyroRange::DPS_125:
                return OSCP_GYRO_DR_125DPS;

            case GyroRange::DPS_250:
                return OSCP_GYRO_DR_250DPS;

            case GyroRange::DPS_500:
                return OSCP_GYRO_DR_500DPS;

            case GyroRange::DPS_1000:
                return OSCP_GYRO_DR_1000DPS;

            case GyroRange::DPS_2000:
                return OSCP_GYRO_DR_2000DPS;

            case GyroRange::DPS_4000:
                return OSCP_GYRO_DR_4000DPS;

            default:
                throw std::runtime_error("Invalid gyro range");
        }
    }

    inline oscp_accel_dr_t to_oscp(AccelRange r) {
        switch(r) {
            case AccelRange::G_2:
                return OSCP_ACCEL_DR_2G;

            case AccelRange::G_4:
                return OSCP_ACCEL_DR_4G;

            case AccelRange::G_8:
                return OSCP_ACCEL_DR_8G;

            case AccelRange::G_16:
                return OSCP_ACCEL_DR_16G;

            default:
                throw std::runtime_error("Invalid accel range");
        }
    }

    inline oscp_incl_dr_t to_oscp(InclRange r) {
        switch(r) {
            case InclRange::G_0_5:
                return OSCP_INCL_DR_0G5;

            case InclRange::G_1_0:
                return OSCP_INCL_DR_1G0;

            case InclRange::G_2_0:
                return OSCP_INCL_DR_2G0;

            case InclRange::G_3_0:
                return OSCP_INCL_DR_3G0;

            default:
                throw std::runtime_error("Invalid inclinometer range");
        }  
    }

    inline uint8_t to_oscp(FilterMode mode) {
        switch(mode) {
            case FilterMode::DISABLED:
                return 0;

            case FilterMode::LP_ONLY:
                return 1;

            case FilterMode::HP_ONLY:
                return 2;

            case FilterMode::LP_AND_HP:
                return 3;

            default:
                throw std::runtime_error("Invalid filter mode");
        }
    }

    inline uint8_t to_oscp(FilterCutoff cutoff) {
        switch(cutoff) {
            case FilterCutoff::C0: 
                return 0;

            case FilterCutoff::C1:
                return 1;

            case FilterCutoff::C2:
                return 2;

            case FilterCutoff::C3: 
                return 3;

            case FilterCutoff::C4: 
                return 4;

            case FilterCutoff::C5: 
                return 5;

            case FilterCutoff::C6: 
                return 6;

            case FilterCutoff::C7: 
                return 7;

            default:
                throw std::runtime_error("Invalid filter cutoff");
        }
    }

    inline uint8_t operatingModeToStartup(OperatingMode mode) {
        switch(mode) {
            case OperatingMode::IDLE:
                return 0;

            case OperatingMode::LOW:
                return 1;

            case OperatingMode::MEDIUM:
                return 2;

            default:
                throw std::runtime_error("Invalid operating mode");
        }
    }   

    inline uint8_t gyroRangeToStartup(GyroRange range) {
        switch(range) {
            case GyroRange::DPS_125:
                return 2;

            case GyroRange::DPS_250:
                return 0;

            case GyroRange::DPS_500:
                return 4;

            case GyroRange::DPS_1000:
                return 8;

            case GyroRange::DPS_2000:
                return 12;

            case GyroRange::DPS_4000:
                return 1;

            default:
                throw std::runtime_error("Invalid gyro range");
        }
    }


    inline uint8_t accelRangeToStartup(AccelRange range) {
        switch(range) {
            case AccelRange::G_2:
                return 0;

            case AccelRange::G_4:
                return 2;

            case AccelRange::G_8:
                return 3;

            case AccelRange::G_16:
                return 1;

            default:
                throw std::runtime_error("Invalid accel range");
        }
    }

    inline uint8_t inclRangeToStartup(InclRange range) {
        switch(range) {
            case InclRange::G_0_5:
                return 0;

            case InclRange::G_1_0:
                return 2;

            case InclRange::G_2_0:
                return 3;

            case InclRange::G_3_0:
                return 1;

            default:
                throw std::runtime_error("Invalid inclinometer range");
        }
    }

    inline uint8_t filterModeToStartup(FilterMode mode) {
        switch(mode) {
            case FilterMode::DISABLED:
                return 0;

            case FilterMode::LP_ONLY:
                return 1;

            case FilterMode::HP_ONLY:
                return 2;

            case FilterMode::LP_AND_HP:
                return 3;

            default:
                throw std::runtime_error("Invalid filter mode");
        }
    }


    inline uint8_t filterCutoffToStartup(FilterCutoff cutoff) {
        switch(cutoff) {
            case FilterCutoff::C0:
                return 0;

            case FilterCutoff::C1:
                return 1;

            case FilterCutoff::C2:
                return 2;

            case FilterCutoff::C3:
                return 3;

            case FilterCutoff::C4:
                return 4;

            case FilterCutoff::C5:
                return 5;

            case FilterCutoff::C6:
                return 6;

            case FilterCutoff::C7:
                return 7;
                
            default:
                throw std::runtime_error("Invalid filter cutoff");
        }
    }

} // namespace oscp_imu

#endif // OSCP_IMU_ROS2_OSCP_IMU_ROSPARAM_HPP