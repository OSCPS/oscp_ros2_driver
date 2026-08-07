#ifndef OSCP_IMU_ROS2_OSCP_IMU_NODE_HPP
#define OSCP_IMU_ROS2_OSCP_IMU_NODE_HPP

/* Includes */
// Standards includes
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <cstdint>
#include <tf2/LinearMath/Quaternion.h>

// ROS Messages includes
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>

// OSCP Messages includes
#include <oscp_msgs/msg/oscp_raw.hpp>
#include <oscp_msgs/msg/oscp_rot.hpp>
#include <oscp_msgs/msg/oscp_quat.hpp>
#include <oscp_msgs/msg/oscp_euler.hpp>
#include <oscp_msgs/msg/oscp_gnss.hpp>

// OSCP Specific Includes
#include "oscp_imu_ros2/oscp_imu_config.hpp"
#include "oscp_imu_ros2/mag_calibration_helper.hpp"
#include "oscp_imu_ros2/oscp_imu_startup_info.hpp"

// Actions and services includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <oscp_imu_ros2/srv/stationary_calibrate.hpp>
#include <oscp_imu_ros2/srv/zero_orientation.hpp>
#include "oscp_imu_ros2/srv/get_imu_config.hpp"
#include <motion_cal_wrapper/action/magnetometer_calibration.hpp>

extern "C" {
#include "oscp_imu.h"
}

/* OSCP ROS2 Node */
class OSCPIMUNode : public rclcpp::Node {

public:
    // Namespace for Less Verbosity
    using MagnetometerCalibration = motion_cal_wrapper::action::MagnetometerCalibration;
    using GoalHandleMagCal = rclcpp_action::ServerGoalHandle<MagnetometerCalibration>;

    /* Constructor */
    OSCPIMUNode();

    /* Destructor */
    ~OSCPIMUNode();

private:
    /* Variables */
    // IMU Communication
    std::thread serial_thread_;
    std::atomic<bool> stop_serial_thread_{false};
    int fd_{-1};
    uint8_t buf_[2048];

    // IMU Config
    oscp_imu::OSCPIMUConfig config_;

    // Startup Frame
    std::atomic<bool> startup_received_{false};
    oscp_imu::StartupInfo startup_info_;

    // Parsing Logic
    oscp_parser_t parser_;

    oscp_raw_t latest_raw_;
    oscp_quat_t latest_quat_;
    oscp_euler_t latest_euler_;
    oscp_rot_mat_t latest_rotation_matrix_;
    oscp_gnss_t latest_gnss_;

    bool has_raw_{false};
    bool has_quat_{false};
    bool has_euler_{false};
    bool has_rotation_matrix_{false};
    bool has_gnss_{false};

    uint32_t last_raw_counter_ = 0;
    uint32_t last_quat_counter_ = 0;
    uint32_t last_euler_counter_ = 0;
    uint32_t last_rot_counter_ = 0;
    uint32_t last_gnss_counter_ = 0;
    uint32_t last_ros_counter_ = 0;

    uint8_t debug_counter = 0;

    // Watchdog
    std::chrono::steady_clock::time_point last_frame_time_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr parser_stats_timer_;
    double watchdog_timeout_ms_{10000.0};  // milliseconds before shutdown
    double parser_stats_log_interval_s_{5.0};

    // Magnetometer Calibration
    oscp_imu::MagCalibrationHelper mag_calibrator_;

    // Publishers
    rclcpp::TimerBase::SharedPtr timer_;
    bool publish_enable_ = false;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpRaw>::SharedPtr raw_frame_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpQuat>::SharedPtr quat_frame_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpEuler>::SharedPtr euler_frame_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpRot>::SharedPtr rot_frame_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpGnss>::SharedPtr gnss_frame_pub_;

    std::mutex frame_mutex_;

    // Zeroing Orientation Service
    tf2::Quaternion current_orientation_;
    oscp_quat_t zero_orientation_offset_;
    bool zero_orientation_set_{false};

    bool euler_zeroed_{false};
    double zero_roll_{0.0};
    double zero_pitch_{0.0};
    double zero_yaw_{0.0};

    oscp_rot_mat_t zero_rotation_matrix_;
    bool rotation_zeroed_{false};

    // Calibration Service
    std::atomic<bool> stationary_calibration_active_{false};
    std::atomic<bool> mag_calibrating_{false};

    // Calibration Threads
    std::thread stationary_calibration_thread_;
    std::thread mag_calibration_thread_;

    /* Functions */
    // Transport Functions
    void init_rs422(int fd);
    void init_canfd(int fd);
    speed_t baud_to_termios(int baud);
    void read_transport();

    // IMU Functions
    bool send_command(oscp_err_t result, uint8_t *cmd, size_t cmd_len);
    bool configure_imu();
    bool write_float_register(oscp_usr_reg_t reg, float value);
    bool verify_startup_config();

    static void on_frame_trampoline(const oscp_frame_t *f, void *ctx);
    void on_frame(const oscp_frame_t *f);

    // Utility Functions
    uint32_t float_to_u32(float value);
    double normalize_degrees(double angle);
    
    // Publishers Functions
    void publish();
    void publish_raw_frame();
    void publish_quat_frame();
    void publish_euler_frame();
    void publish_rot_frame();
    void publish_gnss_frame();
    void publish_imu();
    void publish_magnetometer();
    void publish_temperature();

    // Service Functions
    void get_config_callback(const std::shared_ptr<oscp_imu_ros2::srv::GetIMUConfig::Request> request,std::shared_ptr<oscp_imu_ros2::srv::GetIMUConfig::Response> response);
    
    bool stationary_calibrate(double duration);
    void stationary_calibration_worker(double duration);
    void stationary_calibrate_callback(const std::shared_ptr<oscp_imu_ros2::srv::StationaryCalibrate::Request> request, std::shared_ptr<oscp_imu_ros2::srv::StationaryCalibrate::Response> response);

    void zero_rotation(float current[3][3], float corrected[3][3]);
    void zero_orientation_callback(const std::shared_ptr<oscp_imu_ros2::srv::ZeroOrientation::Request> request, std::shared_ptr<oscp_imu_ros2::srv::ZeroOrientation::Response> response);

    rclcpp::Service<oscp_imu_ros2::srv::StationaryCalibrate>::SharedPtr stationary_calibration_service_;
    rclcpp::Service<oscp_imu_ros2::srv::ZeroOrientation>::SharedPtr zero_orientation_service_;
    rclcpp::Service<oscp_imu_ros2::srv::GetIMUConfig>::SharedPtr config_service_;
    
    // Action Functions
    void handle_mag_accept(const std::shared_ptr<GoalHandleMagCal> goal_handle);
    void execute_mag_calibration( const std::shared_ptr<GoalHandleMagCal> goal_handle);   

    rclcpp_action::Server<MagnetometerCalibration>::SharedPtr mag_calibration_server_;
    rclcpp_action::GoalResponse handle_mag_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MagnetometerCalibration::Goal> goal);
    rclcpp_action::CancelResponse handle_mag_cancel(const std::shared_ptr<GoalHandleMagCal> goal_handle);
};

#endif // OSCP_IMU_ROS2_OSCP_IMU_NODE_HPP
