#pragma once

#include <string>
#include <stdexcept>

#include "oscp_imu_ros2/oscp_imu_config.hpp"

namespace imu
{

inline Transport parseTransport(const std::string &s)
{
    if (s == "RS422") return Transport::RS422;
    if (s == "CANFD") return Transport::CANFD;

    throw std::runtime_error("Invalid transport: " + s);
}

    inline OperatingMode parseOperatingMode(const std::string & s)
{
    if (s == "IDLE") return OperatingMode::IDLE;
    if (s == "LOW") return OperatingMode::LOW;
    if (s == "MEDIUM") return OperatingMode::MEDIUM;

    throw std::runtime_error("Invalid operating_mode: " + s);
}

inline AccelRange parseAccelRange(const std::string &s)
{
    if (s == "G_2") return AccelRange::G_2;
    if (s == "G_4") return AccelRange::G_4;
    if (s == "G_8") return AccelRange::G_8;
    if (s == "G_16") return AccelRange::G_16;
    return AccelRange::G_4;
}

inline InclRange parseInclRange(const std::string &s)
{
    if (s == "G_0_5") return InclRange::G_0_5;
    if (s == "G_1_0") return InclRange::G_1_0;
    if (s == "G_2_0") return InclRange::G_2_0;
    if (s == "G_3_0") return InclRange::G_3_0;
    return InclRange::G_1_0;
}

inline FilterMode parseFilterMode(const std::string &s)
{
    if (s == "DISABLED") return FilterMode::DISABLED;
    if (s == "LP_ONLY") return FilterMode::LP_ONLY;
    if (s == "HP_ONLY") return FilterMode::HP_ONLY;
    if (s == "LP_AND_HP") return FilterMode::LP_AND_HP;
    return FilterMode::LP_ONLY;
}

inline FilterCutoff parseFilterCutoff(const std::string &s)
{
    if (s == "C0") return FilterCutoff::C0;
    if (s == "C1") return FilterCutoff::C1;
    if (s == "C2") return FilterCutoff::C2;
    if (s == "C3") return FilterCutoff::C3;
    if (s == "C4") return FilterCutoff::C4;
    if (s == "C5") return FilterCutoff::C5;
    if (s == "C6") return FilterCutoff::C6;
    if (s == "C7") return FilterCutoff::C7;
    return FilterCutoff::C0;
}

inline AHRSHeadingSource parseAHRSHeadingSource(const std::string& s)
{
    if (s == "NONE") return AHRSHeadingSource::NONE;
    if (s == "INTERNAL_MAGNETOMETER") return AHRSHeadingSource::INTERNAL_MAGNETOMETER;

    return AHRSHeadingSource::NONE;
}

inline AHRSConvention parseAHRSConvention(const std::string& s)
{
    if (s == "NWU") return AHRSConvention::NWU;
    if (s == "ENU") return AHRSConvention::ENU;
    if (s == "NED") return AHRSConvention::NED;

    return AHRSConvention::NWU; // safe default
}

inline GyroRange parseGyroRange(const std::string & s)
{
    if (s == "DPS_125") return GyroRange::DPS_125;
    if (s == "DPS_250") return GyroRange::DPS_250;
    if (s == "DPS_500") return GyroRange::DPS_500;
    if (s == "DPS_1000") return GyroRange::DPS_1000;
    if (s == "DPS_2000") return GyroRange::DPS_2000;
    if (s == "DPS_4000") return GyroRange::DPS_4000;

    throw std::runtime_error("Invalid gyro_range: " + s);
}

inline std::string toString(OperatingMode m)
{
    switch (m)
    {
        case OperatingMode::IDLE:   return "IDLE";
        case OperatingMode::LOW:    return "LOW";
        case OperatingMode::MEDIUM: return "MEDIUM";
    }
    return "IDLE";
}

inline std::string toString(Transport t)
{
    switch(t)
    {
        case Transport::RS422: return "RS422";
        case Transport::CANFD: return "CANFD";
    }

    return "RS422";
}

inline std::string toString(GyroRange r)
{
    switch (r)
    {
        case GyroRange::DPS_125:  return "DPS_125";
        case GyroRange::DPS_250:  return "DPS_250";
        case GyroRange::DPS_500:  return "DPS_500";
        case GyroRange::DPS_1000: return "DPS_1000";
        case GyroRange::DPS_2000: return "DPS_2000";
        case GyroRange::DPS_4000: return "DPS_4000";
    }
    return "DPS_250";
}

inline std::string toString(AccelRange r)
{
    switch (r)
    {
        case AccelRange::G_2:  return "G_2";
        case AccelRange::G_4:  return "G_4";
        case AccelRange::G_8:  return "G_8";
        case AccelRange::G_16: return "G_16";
    }
    return "G_4";
}

inline std::string toString(InclRange r)
{
    switch (r)
    {
        case InclRange::G_0_5: return "G_0_5";
        case InclRange::G_1_0: return "G_1_0";
        case InclRange::G_2_0: return "G_2_0";
        case InclRange::G_3_0: return "G_3_0";
    }
    return "G_1_0";
}

inline std::string toString(FilterMode m)
{
    switch (m)
    {
        case FilterMode::DISABLED:  return "DISABLED";
        case FilterMode::LP_ONLY:   return "LP_ONLY";
        case FilterMode::HP_ONLY:   return "HP_ONLY";
        case FilterMode::LP_AND_HP: return "LP_AND_HP";
    }
    return "LP_ONLY";
}

inline std::string toString(FilterCutoff c)
{
    switch (c)
    {
        case FilterCutoff::C0: return "C0";
        case FilterCutoff::C1: return "C1";
        case FilterCutoff::C2: return "C2";
        case FilterCutoff::C3: return "C3";
        case FilterCutoff::C4: return "C4";
        case FilterCutoff::C5: return "C5";
        case FilterCutoff::C6: return "C6";
        case FilterCutoff::C7: return "C7";
    }
    return "C0";
}

inline std::string toString(AHRSConvention c)
{
    switch (c)
    {
        case AHRSConvention::NWU: return "NWU";
        case AHRSConvention::ENU: return "ENU";
        case AHRSConvention::NED: return "NED";
    }
    return "NWU";
}

inline std::string toString(AHRSHeadingSource s)
{
    switch (s)
    {
        case AHRSHeadingSource::NONE: return "NONE";
        case AHRSHeadingSource::INTERNAL_MAGNETOMETER: return "INTERNAL_MAGNETOMETER";
    }
    return "NONE";
}

inline uint32_t toOSCP(imu::AHRSConvention c)
{
    switch(c)
    {
        case imu::AHRSConvention::NWU:
            return 0;

        case imu::AHRSConvention::ENU:
            return 1;

        case imu::AHRSConvention::NED:
            return 2;
    }

    return 0;
}


inline uint32_t toOSCP(imu::AHRSHeadingSource h)
{
    switch(h)
    {
        case imu::AHRSHeadingSource::NONE:
            return 0;

        case imu::AHRSHeadingSource::INTERNAL_MAGNETOMETER:
            return 1;
    }

    return 0;
}


} // namespace imu