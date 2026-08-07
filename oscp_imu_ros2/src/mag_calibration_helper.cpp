/* Includes */
#include <cmath>
#include <cstddef>
#include <vector>

#include "oscp_imu_ros2/mag_calibration_helper.hpp"

/* Functions */
oscp_imu::MagCalibrationHelper::Estimate oscp_imu::MagCalibrationHelper::estimate_bias(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z) {
    Estimate estimate{};
    if (x.empty() || y.empty() || z.empty()) {
        return estimate;
    }

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    for (size_t i = 0; i < x.size(); ++i) {
        sum_x += x[i];
        sum_y += y[i];
        sum_z += z[i];
    }

    estimate.offset_x = sum_x / static_cast<float>(x.size());
    estimate.offset_y = sum_y / static_cast<float>(y.size());
    estimate.offset_z = sum_z / static_cast<float>(z.size());
    return estimate;
}

float oscp_imu::MagCalibrationHelper::compute_gap(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z, float offset_x, float offset_y, float offset_z) {
    if (x.empty() || y.empty() || z.empty()) {
        return 9999.0f;
    }

    float max_gap = 0.0f;
    for (size_t i = 0; i < x.size(); ++i) {
        float corrected_x = x[i] - offset_x;
        float corrected_y = y[i] - offset_y;
        float corrected_z = z[i] - offset_z;
        float magnitude = std::sqrt(corrected_x * corrected_x + corrected_y * corrected_y + corrected_z * corrected_z);
        max_gap = std::max(max_gap, magnitude);
    }

    return max_gap;
}

oscp_imu::MagCalibrationHelper::Calibration oscp_imu::MagCalibrationHelper::solve(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &z) {
    Calibration calibration{};
    if (x.empty() || y.empty() || z.empty()) {
        return calibration;
    }

    Estimate estimate = estimate_bias(x, y, z);
    calibration.offset_x = estimate.offset_x;
    calibration.offset_y = estimate.offset_y;
    calibration.offset_z = estimate.offset_z;

    float max_mag = 0.0f;
    for (size_t i = 0; i < x.size(); ++i) {
        float corrected_x = x[i] - calibration.offset_x;
        float corrected_y = y[i] - calibration.offset_y;
        float corrected_z = z[i] - calibration.offset_z;
        float magnitude = std::sqrt(corrected_x * corrected_x + corrected_y * corrected_y + corrected_z * corrected_z);
        max_mag = std::max(max_mag, magnitude);
    }

    calibration.fit_error = max_mag > 0.0f ? max_mag : 0.0f;
    return calibration;
}
