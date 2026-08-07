#ifndef OSCP_IMU_ROS2_MAG_CALIBRATION_HELPER_HPP
#define OSCP_IMU_ROS2_MAG_CALIBRATION_HELPER_HPP
#endif // OSCP_IMU_ROS2_MAG_CALIBRATION_HELPER_HPP

/* Includes */
#include <array>
#include <vector>
#include <mutex>

namespace oscp_imu {

class MagCalibrationHelper {

public:
    /* Struct */
    struct Estimate {
        float offset_x{0.0f};
        float offset_y{0.0f};
        float offset_z{0.0f};
    };

    struct Calibration {
        float offset_x{0.0f};
        float offset_y{0.0f};
        float offset_z{0.0f};
        float fit_error{0.0f};
        std::array<std::array<float, 3>, 3> soft_iron{{{1.0f, 0.0f, 0.0f},
                                                        {0.0f, 1.0f, 0.0f},
                                                        {0.0f, 0.0f, 1.0f}}};
    };
    
    /* Variables */
    float target_gap{0.1f};
    float current_gap{2.0f};
    std::vector<float> mag_x_samples;
    std::vector<float> mag_y_samples;
    std::vector<float> mag_z_samples;
    std::mutex mag_samples_mutex;

    /* Functions */
    Estimate estimate_bias(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z);

    float compute_gap(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z, float offset_x, float offset_y, float offset_z);

    Calibration solve(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z);
}; 

} // namespace oscp_imu::MagCalibrationHelper