#pragma once

#include "oscp_imu_ros2/oscp_imu_config.hpp"
#include "oscp_imu.h"


inline oscp_transport_t toOSCP(imu::Transport t)
{
    switch(t)
    {
        case imu::Transport::RS422:
            return OSCP_TRANSPORT_RS422;

        case imu::Transport::CANFD:
            return OSCP_TRANSPORT_CANFD;
    }

    return OSCP_TRANSPORT_RS422;
}


inline oscp_om_sel_t toOSCP(imu::OperatingMode m)
{
    switch(m)
    {
        case imu::OperatingMode::IDLE:
            return OSCP_OM_IDLE;

        case imu::OperatingMode::LOW:
            return OSCP_OM_LOW;

        case imu::OperatingMode::MEDIUM:
            return OSCP_OM_MEDIUM;
    }

    return OSCP_OM_LOW;
}


inline oscp_gyro_dr_t toOSCP(imu::GyroRange r)
{
    switch(r)
    {
        case imu::GyroRange::DPS_125:
            return OSCP_GYRO_DR_125DPS;

        case imu::GyroRange::DPS_250:
            return OSCP_GYRO_DR_250DPS;

        case imu::GyroRange::DPS_500:
            return OSCP_GYRO_DR_500DPS;

        case imu::GyroRange::DPS_1000:
            return OSCP_GYRO_DR_1000DPS;

        case imu::GyroRange::DPS_2000:
            return OSCP_GYRO_DR_2000DPS;

        case imu::GyroRange::DPS_4000:
            return OSCP_GYRO_DR_4000DPS;
    }

    return OSCP_GYRO_DR_500DPS;
}


inline oscp_accel_dr_t toOSCP(imu::AccelRange r)
{
    switch(r)
    {
        case imu::AccelRange::G_2:
            return OSCP_ACCEL_DR_2G;

        case imu::AccelRange::G_4:
            return OSCP_ACCEL_DR_4G;

        case imu::AccelRange::G_8:
            return OSCP_ACCEL_DR_8G;

        case imu::AccelRange::G_16:
            return OSCP_ACCEL_DR_16G;
    }

    return OSCP_ACCEL_DR_4G;
}

inline oscp_incl_dr_t toOSCP(imu::InclRange r)
{
    switch(r)
    {
        case imu::InclRange::G_0_5:
            return OSCP_INCL_DR_0G5;

        case imu::InclRange::G_1_0:
            return OSCP_INCL_DR_1G0;

        case imu::InclRange::G_2_0:
            return OSCP_INCL_DR_2G0;

        case imu::InclRange::G_3_0:
            return OSCP_INCL_DR_3G0;
    }

    return OSCP_INCL_DR_1G0;
}

inline uint8_t toOSCP(imu::FilterMode mode)
{
    switch(mode)
    {
        case imu::FilterMode::DISABLED:
            return 0;

        case imu::FilterMode::LP_ONLY:
            return 1;

        case imu::FilterMode::HP_ONLY:
            return 2;

        case imu::FilterMode::LP_AND_HP:
            return 3;
    }

    return 1;
}

inline uint8_t toOSCP(imu::FilterCutoff cutoff)
{
    switch(cutoff)
    {
        case imu::FilterCutoff::C0: return 0;
        case imu::FilterCutoff::C1: return 1;
        case imu::FilterCutoff::C2: return 2;
        case imu::FilterCutoff::C3: return 3;
        case imu::FilterCutoff::C4: return 4;
        case imu::FilterCutoff::C5: return 5;
        case imu::FilterCutoff::C6: return 6;
        case imu::FilterCutoff::C7: return 7;
    }

    return 3;
}

