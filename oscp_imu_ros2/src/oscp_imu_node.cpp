/* Includes */
// Standards includes
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <cstring>
#include <chrono>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// OSCP Specific Includes
#include "oscp_imu_ros2/oscp_imu_param_translator.hpp"
#include "oscp_imu_ros2/oscp_imu_rosparam.hpp"
#include "oscp_imu_ros2/oscp_imu_node.hpp"

/* Constuctor */
OSCPIMUNode::OSCPIMUNode() : rclcpp::Node("oscp_imu_node") {
    // Default values for initialization that wont work without being set by the user
    declare_parameter("device", std::string(""));
    declare_parameter("baudrate", -1);
    declare_parameter("frame_id", std::string(""));
    declare_parameter("pub_standard_ros_accel_in_g", false);
    declare_parameter("transport",oscp_imu::toString(config_.transport));

    declare_parameter("operating_mode", oscp_imu::toString(config_.operating_mode));

    declare_parameter("gyro_range", oscp_imu::toString(config_.gyro_range));
    declare_parameter("accel_range", oscp_imu::toString(config_.accel_range));
    declare_parameter("incl_range", oscp_imu::toString(config_.incl_range));

    declare_parameter("gyro_filter_mode", oscp_imu::toString(config_.gyro_filter_mode));
    declare_parameter("gyro_lpf", oscp_imu::toString(config_.gyro_lpf));
    declare_parameter("gyro_hpf", oscp_imu::toString(config_.gyro_hpf));

    declare_parameter("misalignment_correction", config_.misalignment_correction);

    declare_parameter("accel_filter_mode", oscp_imu::toString(config_.accel_filter_mode));
    declare_parameter("accel_lpf", oscp_imu::toString(config_.accel_lpf));
    declare_parameter("accel_hpf", oscp_imu::toString(config_.accel_hpf));

    declare_parameter("ahrs_convention", oscp_imu::toString(config_.ahrs_convention));
    declare_parameter("ahrs_heading_source", oscp_imu::toString(config_.ahrs_heading));
    
    declare_parameter<bool>("publish_standard_ros", config_.publish_standard_ros);
    declare_parameter<bool>("publish_oscp_raw", config_.publish_oscp_raw);
    declare_parameter<bool>("publish_oscp_euler", config_.publish_oscp_euler);
    declare_parameter<bool>("publish_oscp_quat", config_.publish_oscp_quat);
    declare_parameter<bool>("publish_oscp_rotation_matrix", config_.publish_oscp_rot);
    declare_parameter<bool>("publish_oscp_gnss", config_.publish_oscp_gnss);

    declare_parameter<double>("watchdog_timeout_ms", config_.watchdog_timeout_ms_);
    declare_parameter<double>("parser_stats_log_interval_s", parser_stats_log_interval_s_);

    config_.device = this->get_parameter("device").as_string();
    config_.baudrate = static_cast<int>(this->get_parameter("baudrate").as_int());
    config_.frame_id = this->get_parameter("frame_id").as_string();
    config_.pub_standard_ros_accel_in_g = this->get_parameter("pub_standard_ros_accel_in_g").as_bool();

    if (config_.device.empty() || config_.baudrate < 0 || config_.frame_id.empty()) {
        RCLCPP_FATAL(get_logger(), "Required parameters not set — set in launch file");
        throw std::runtime_error("Missing required parameters");
    }

    config_.operating_mode = oscp_imu::parseOperatingMode(get_parameter("operating_mode").as_string());
    config_.transport = oscp_imu::parseTransport(get_parameter("transport").as_string());
    config_.gyro_range = oscp_imu::parseGyroRange(get_parameter("gyro_range").as_string());
    config_.accel_range = oscp_imu::parseAccelRange(get_parameter("accel_range").as_string());
    config_.incl_range = oscp_imu::parseInclRange(get_parameter("incl_range").as_string());
    config_.misalignment_correction = get_parameter("misalignment_correction").as_bool();
    config_.gyro_filter_mode = oscp_imu::parseFilterMode(get_parameter("gyro_filter_mode").as_string());
    config_.gyro_lpf = oscp_imu::parseFilterCutoff(get_parameter("gyro_lpf").as_string());
    config_.gyro_hpf = oscp_imu::parseFilterCutoff(get_parameter("gyro_hpf").as_string());
    config_.accel_filter_mode = oscp_imu::parseFilterMode(get_parameter("accel_filter_mode").as_string());
    config_.accel_lpf = oscp_imu::parseFilterCutoff(get_parameter("accel_lpf").as_string());
    config_.accel_hpf = oscp_imu::parseFilterCutoff(get_parameter("accel_hpf").as_string());
    config_.ahrs_convention = oscp_imu::parseAHRSConvention(get_parameter("ahrs_convention").as_string());
    config_.ahrs_heading = oscp_imu::parseAHRSHeadingSource(get_parameter("ahrs_heading_source").as_string());

    config_.publish_standard_ros = this->get_parameter("publish_standard_ros").as_bool();
    config_.publish_oscp_raw = this->get_parameter("publish_oscp_raw").as_bool();
    config_.publish_oscp_euler = this->get_parameter("publish_oscp_euler").as_bool();
    config_.publish_oscp_quat = this->get_parameter("publish_oscp_quat").as_bool();
    config_.publish_oscp_rot = this->get_parameter("publish_oscp_rotation_matrix").as_bool();
    config_.publish_oscp_gnss = this->get_parameter("publish_oscp_gnss").as_bool();

    config_.watchdog_timeout_ms_ = this->get_parameter("watchdog_timeout_ms").as_double();
    parser_stats_log_interval_s_ = this->get_parameter("parser_stats_log_interval_s").as_double();

    /* Buffering more sample for Medium Operating Mode use-cases */
    rclcpp::QoS high_rate_qos = rclcpp::SensorDataQoS();
    high_rate_qos.keep_last(1000);

    // Standard ROS publishers
    if (config_.publish_standard_ros) {
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("oscp/imu/data", high_rate_qos);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("oscp/imu/mag", high_rate_qos);
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("oscp/imu/temp", high_rate_qos);
    }

    // OSCP frame publishers
    if (config_.publish_oscp_raw) {
        raw_frame_pub_ = create_publisher<oscp_msgs::msg::OscpRaw>("oscp/raw", high_rate_qos);
    }

    if (config_.publish_oscp_quat) {
        quat_frame_pub_ = create_publisher<oscp_msgs::msg::OscpQuat>("oscp/quat", high_rate_qos);
    }

    if (config_.publish_oscp_euler) {
        euler_frame_pub_ = create_publisher<oscp_msgs::msg::OscpEuler>("oscp/euler", high_rate_qos);
    }

    if (config_.publish_oscp_rot) {
        rot_frame_pub_ = create_publisher<oscp_msgs::msg::OscpRot>("oscp/rot", high_rate_qos);
    }

    if (config_.publish_oscp_gnss) {
        gnss_frame_pub_ = create_publisher<oscp_msgs::msg::OscpGnss>("oscp/gnss", high_rate_qos);
    }

    oscp_parser_init(&parser_, &OSCPIMUNode::on_frame_trampoline, this);

    // Service Initiliazations
    stationary_calibration_service_ = create_service<oscp_imu_ros2::srv::StationaryCalibrate>("oscp/stationary_calibrate",std::bind(&OSCPIMUNode::stationary_calibrate_callback,this,std::placeholders::_1,std::placeholders::_2));
    zero_orientation_service_ = create_service<oscp_imu_ros2::srv::ZeroOrientation>("oscp/zero_orientation",std::bind(&OSCPIMUNode::zero_orientation_callback,this,std::placeholders::_1,std::placeholders::_2));
    config_service_ = this->create_service<oscp_imu_ros2::srv::GetIMUConfig>("/oscp/get_config",std::bind(&OSCPIMUNode::get_config_callback,this,std::placeholders::_1,std::placeholders::_2));

    // Action Server Initialization
    mag_calibration_server_ = rclcpp_action::create_server<MagnetometerCalibration>(this,"oscp/magnetometer_calibration", std::bind(&OSCPIMUNode::handle_mag_goal,this,std::placeholders::_1,std::placeholders::_2),
    std::bind(&OSCPIMUNode::handle_mag_cancel,this,std::placeholders::_1), std::bind(&OSCPIMUNode::handle_mag_accept,this,std::placeholders::_1));

    // Internal Functions
    if (config_.transport == oscp_imu::Transport::CANFD) {
        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    } else {
        fd_ = open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    }

    if (fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", config_.device.c_str());
        return;
    }

    if(config_.transport == oscp_imu::Transport::RS422) {
        init_rs422(fd_);
    } else if (config_.transport == oscp_imu::Transport::CANFD) {
        init_canfd(fd_);
    }

    std::chrono::milliseconds read_period;

    switch(config_.operating_mode) {
        case oscp_imu::OperatingMode::LOW:
            read_period = std::chrono::milliseconds(10);
            break;
        case oscp_imu::OperatingMode::MEDIUM:
            read_period = std::chrono::milliseconds(1);
            break;
        default:
            read_period = std::chrono::milliseconds(10);
            break;
    }

    stop_serial_thread_.store(false);
    serial_thread_ = std::thread([this, read_period]() {
        while (!stop_serial_thread_.load()) {
            read_transport();
        }
    });

    // Let the unit power up (600ms) 
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    // Reset the unit 
    uint8_t cmd[256];
    size_t cmd_len = 0;

    send_command(oscp_cmd_reset(cmd, sizeof(cmd), &cmd_len, to_oscp(config_.transport)), cmd, cmd_len);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Configure the IMU with the parameters set in the launch file
    publish_enable_ = configure_imu();

    if (!publish_enable_) {
        RCLCPP_ERROR(get_logger(),"IMU configuration failed - Node Shutdown - Verify Setup");
        rclcpp::shutdown();
    } else {
        RCLCPP_INFO(get_logger(),"IMU node started on %s", config_.device.c_str());
        RCLCPP_INFO(get_logger(),"Publishing Topics...");

        // Watchdog — kills node if no frames received after startup
        last_frame_time_ = std::chrono::steady_clock::now();

        watchdog_timer_ = this->create_wall_timer(std::chrono::seconds(1),
            [this]() {
                if (!startup_received_.load()) return;

                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time_).count();

                if (elapsed > config_.watchdog_timeout_ms_) {
                    RCLCPP_FATAL(get_logger(), "No frames received for %ld ms, shutting down node", elapsed);
                    rclcpp::shutdown();
                }
            });

        if (parser_stats_log_interval_s_ > 0.0) {
            parser_stats_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(parser_stats_log_interval_s_),
                [this]() {
                    const oscp_stats_t *stats = oscp_parser_stats(&parser_);
                    if (stats == nullptr) {
                        return;
                    }

                    RCLCPP_INFO(
                        get_logger(),
                        "Parser stats: ok=%u framing_errors=%u crc_errors=%u cobs_errors=%u overflows=%u",
                        stats->frames_ok,
                        stats->framing_errors,
                        stats->crc_errors,
                        stats->cobs_errors,
                        stats->overflows);
                });
        }
    }
}

/* Destructor */
OSCPIMUNode::~OSCPIMUNode() {
    stop_serial_thread_.store(true);
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }

    if (mag_calibration_thread_.joinable()) {
        mag_calibration_thread_.join();
    }

    if (stationary_calibration_thread_.joinable()) {
        stationary_calibration_thread_.join();
    }

    if (fd_ >= 0) {
        close(fd_);
    }
}

/* Transport Functions */
void OSCPIMUNode::init_rs422(int fd) {
    struct termios tty{};
    tcgetattr(fd, &tty);

    speed_t spd = baud_to_termios(config_.baudrate);

    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tcsetattr(fd, TCSANOW, &tty);

    RCLCPP_INFO(get_logger(), "Serial configured: %d baud, 8N1", config_.baudrate);
}

void OSCPIMUNode::init_canfd(int fd) {
    struct ifreq ifr{};
    struct sockaddr_can addr{};

    std::strncpy(ifr.ifr_name, config_.device.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to get CAN interface index for %s: %s",
            config_.device.c_str(), std::strerror(errno));
        rclcpp::shutdown();
        return;
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to bind CAN socket on %s: %s",
            config_.device.c_str(), std::strerror(errno));
        rclcpp::shutdown();
        return;
    }

    int enable_canfd = 1;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to enable CAN-FD on %s: %s",
            config_.device.c_str(), std::strerror(errno));
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(get_logger(), "CAN-FD configured on %s", config_.device.c_str());
}

speed_t OSCPIMUNode::baud_to_termios(int baud) {
    switch (baud) {
        case 9600:
            return B9600;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            RCLCPP_WARN(get_logger(), "Unsupported baud rate %d, defaulting to 921600", baud);
            return B921600;
    }
}

void OSCPIMUNode::read_transport() {
    if (fd_ < 0) {
        return;
    }

    pollfd poll_fd{};
    poll_fd.fd = fd_;
    poll_fd.events = POLLIN;

    int poll_result = ::poll(&poll_fd, 1, 1);
    if (poll_result <= 0) {
        return;
    }

    if ((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        RCLCPP_ERROR(get_logger(), "Transport error while polling");
        return;
    }

    if (config_.transport == oscp_imu::Transport::CANFD) {

        struct canfd_frame frame{};
        ssize_t n = read(fd_, &frame, sizeof(frame));

        if (n == static_cast<ssize_t>(sizeof(canfd_frame))) {
            oscp_parser_feed_buf(&parser_, frame.data, frame.len);
        } else if (n == static_cast<ssize_t>(sizeof(can_frame))) {
            RCLCPP_WARN(get_logger(), "Received classic CAN frame on CAN-FD socket");
        } else if (n < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(get_logger(), "CAN-FD read failed: %s", std::strerror(errno));
        }

    } else if (config_.transport == oscp_imu::Transport::RS422) {

        std::array<uint8_t, 8192> read_buffer{};
        ssize_t n = read(fd_, read_buffer.data(), read_buffer.size());

        if (n > 0) {
            oscp_parser_feed_buf(&parser_, read_buffer.data(), static_cast<size_t>(n));
        } else if (n < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(get_logger(), "Serial read failed: %s", std::strerror(errno));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

    }
}

/* IMU Functions */
bool OSCPIMUNode::send_command(oscp_err_t result, uint8_t *cmd, size_t cmd_len) {
    if(result != OSCP_OK) {
        RCLCPP_ERROR(get_logger(),"Failed to generate IMU command");
        return false;
    }
    ssize_t written = write(fd_, cmd, cmd_len);

    if(written < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to write IMU command");
        return false;
    }

    tcdrain(fd_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return true;
}

bool OSCPIMUNode::configure_imu() {
    uint8_t cmd[256];
    size_t cmd_len = 0;

    // Enter configuration mode
    send_command(oscp_cmd_config(cmd,sizeof(cmd),&cmd_len,to_oscp(config_.transport)),cmd, cmd_len);

    send_command(oscp_cmd_dri(cmd, sizeof(cmd), &cmd_len, to_oscp(config_.incl_range), to_oscp(config_.transport)),cmd, cmd_len);
    send_command(oscp_cmd_disable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_RAW, to_oscp(config_.transport)),cmd, cmd_len);
    send_command(oscp_cmd_disable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_EULER, to_oscp(config_.transport)),cmd, cmd_len);
    send_command(oscp_cmd_disable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_QUATERNION, to_oscp(config_.transport)),cmd, cmd_len);
    send_command(oscp_cmd_disable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_ROT_MATRIX, to_oscp(config_.transport)),cmd, cmd_len);

    send_command(oscp_cmd_om(cmd, sizeof(cmd), &cmd_len, to_oscp(config_.operating_mode), to_oscp(config_.transport)),cmd, cmd_len);
    send_command(oscp_cmd_drg(cmd, sizeof(cmd), &cmd_len, to_oscp(config_.gyro_range), to_oscp(config_.transport)),cmd, cmd_len);
    send_command(oscp_cmd_dra(cmd, sizeof(cmd), &cmd_len, to_oscp(config_.accel_range), to_oscp(config_.transport)),cmd, cmd_len);
    send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len, OSCP_USR_REG_FCO, to_oscp(config_.ahrs_convention), to_oscp(config_.transport)),cmd, cmd_len);
    send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len, OSCP_USR_REG_FHS, to_oscp(config_.ahrs_heading), to_oscp(config_.transport)),cmd, cmd_len);

    // Gyroscope filter configuration
    send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len,OSCP_USR_REG_GFI,to_oscp(config_.gyro_filter_mode),to_oscp(config_.transport)),cmd,cmd_len);
    send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len,OSCP_USR_REG_GLP,to_oscp(config_.gyro_lpf),to_oscp(config_.transport)),cmd,cmd_len);
    send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len,OSCP_USR_REG_GHP,to_oscp(config_.gyro_hpf),to_oscp(config_.transport)),cmd,cmd_len);

    // Accelerometer filter configuration
    send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len,OSCP_USR_REG_AFI,to_oscp(config_.accel_filter_mode),to_oscp(config_.transport)),cmd,cmd_len);
    send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len,OSCP_USR_REG_ALP,to_oscp(config_.accel_lpf),to_oscp(config_.transport)),cmd,cmd_len);
    send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len,OSCP_USR_REG_AHP,to_oscp(config_.accel_hpf),to_oscp(config_.transport)),cmd,cmd_len);

    if(config_.publish_oscp_raw || config_.publish_standard_ros) { 
        send_command(oscp_cmd_enable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_RAW, to_oscp(config_.transport)),cmd, cmd_len);
        RCLCPP_INFO(get_logger(), "Raw Frame Enabled");
    }

    if(config_.publish_oscp_euler) {
        send_command(oscp_cmd_enable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_EULER, to_oscp(config_.transport)),cmd, cmd_len);
        RCLCPP_INFO(get_logger(), "Euler Frame Enabled");
    }

    if(config_.publish_oscp_quat || config_.publish_standard_ros) {
        send_command(oscp_cmd_enable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_QUATERNION, to_oscp(config_.transport)),cmd, cmd_len);
        RCLCPP_INFO(get_logger(), "Quaternion Frame Enabled");
    }

    if(config_.publish_oscp_rot) {
        send_command(oscp_cmd_enable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_ROT_MATRIX, to_oscp(config_.transport)),cmd, cmd_len);
        RCLCPP_INFO(get_logger(), "Rotation Matrix Frame Enabled");
    }

    if(config_.publish_oscp_gnss) {
        send_command(oscp_cmd_enable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_GNSS, to_oscp(config_.transport)),cmd, cmd_len);
        RCLCPP_INFO(get_logger(), "GNSS Frame Enabled");
    }

    RCLCPP_INFO(get_logger(), "IMU configuration sent");

    startup_received_.store(false);
    
    // Request an SUF for identification 
    send_command(oscp_cmd_suf(cmd, sizeof(cmd), &cmd_len, to_oscp(config_.transport)), cmd, cmd_len);
    read_transport();

    // Exit config mode
    send_command(oscp_cmd_exit(cmd, sizeof(cmd), &cmd_len, to_oscp(config_.transport)),cmd, cmd_len);

    // Look for Startup 
    if (!startup_received_.load()) {
        RCLCPP_ERROR(get_logger(),"No startup frame received");
        return false;
    }
    
    if (!verify_startup_config()) {
        RCLCPP_ERROR(get_logger(),"IMU configuration failed");
        return false;
    }

    RCLCPP_INFO(get_logger(),"IMU configuration suceeded");
    return true;
}

bool OSCPIMUNode::write_float_register(oscp_usr_reg_t reg, float value) {
    uint8_t cmd[256];
    size_t cmd_len = 0;

    oscp_err_t err = oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len,reg, float_to_u32(value), to_oscp(config_.transport));

    if (err != OSCP_OK) {
        RCLCPP_ERROR(this->get_logger(),"Failed to create register write command");
        return false;
    }

    int written = write(fd_, cmd, cmd_len);

    RCLCPP_INFO(get_logger(),"Sent command of %ld bytes",cmd_len);

    if (written != static_cast<int>(cmd_len)) {
        RCLCPP_ERROR(this->get_logger(),"Failed to write register command");
        return false;
    }

    tcdrain(fd_);

    return true;
}    

bool OSCPIMUNode::verify_startup_config() {
    bool ok = true;

    const auto &startup = startup_info_;

    uint8_t expected_mode = operatingModeToStartup(config_.operating_mode);
    uint8_t expected_gyro = gyroRangeToStartup(config_.gyro_range);    
    uint8_t expected_accel = accelRangeToStartup(config_.accel_range);
    uint8_t expected_incl = inclRangeToStartup(config_.incl_range);
    uint8_t expected_gyro_filter = filterModeToStartup(config_.gyro_filter_mode);
    uint8_t expected_gyro_lpf = filterCutoffToStartup(config_.gyro_lpf);
    uint8_t expected_gyro_hpf = filterCutoffToStartup(config_.gyro_hpf);
    uint8_t expected_accel_filter = filterModeToStartup(config_.accel_filter_mode);
    uint8_t expected_accel_lpf = filterCutoffToStartup(config_.accel_lpf);
    uint8_t expected_accel_hpf = filterCutoffToStartup(config_.accel_hpf);
    
    RCLCPP_INFO(
        get_logger(),
        "\n"
        "========== IMU Config Verification ==========\n"
        "Operating Mode:\n"
        "  Expected: %u | Received: %u\n"
        "\n"
        "Gyroscope Range:\n"
        "  Expected: %u | Received: %u\n"
        "\n"
        "Accelerometer Range:\n"
        "  Expected: %u | Received: %u\n"
        "\n"
        "Inclination Range:\n"
        "  Expected: %u | Received: %u\n"
        "\n"
        "Gyro Filter:\n"
        "  Mode: %u (expected %u)\n"
        "  LPF:  %u (expected %u)\n"
        "  HPF:  %u (expected %u)\n"
        "\n"
        "Accel Filter:\n"
        "  Mode: %u (expected %u)\n"
        "  LPF:  %u (expected %u)\n"
        "  HPF:  %u (expected %u)\n"
        "==============================================",
        startup.operating_mode, expected_mode, startup.gyro_range, expected_gyro, startup.accel_range, expected_accel, startup.incl_range, expected_incl, startup.gyro_filters, expected_gyro_filter, 
        startup.gyro_lpf, expected_gyro_lpf, startup.gyro_hpf, expected_gyro_hpf, startup.accel_filters, expected_accel_filter, startup.accel_lpf, expected_accel_lpf, startup.accel_hpf, expected_accel_hpf
    );

    if (expected_mode != startup.operating_mode) {
        RCLCPP_ERROR(get_logger(),"Operating mode mismatch");
        ok = false;
    }

    if (expected_gyro != startup.gyro_range) {
        RCLCPP_ERROR(get_logger(),"Gyro range mismatch");
        ok = false;
    }

    if (expected_accel != startup.accel_range) {
        RCLCPP_ERROR(get_logger(),"Accel range mismatch");
        ok = false;
    }

    if (expected_incl != startup.incl_range) {
        RCLCPP_ERROR(get_logger(),"Incl range mismatch");
        ok = false;
    }

    if (expected_gyro_filter != startup.gyro_filters || expected_gyro_lpf != startup.gyro_lpf || expected_gyro_hpf != startup.gyro_hpf) {
        RCLCPP_ERROR(get_logger(),"Gyro filter configuration mismatch");
        ok = false;
    }

    if (expected_accel_filter != startup.accel_filters || expected_accel_lpf != startup.accel_lpf || expected_accel_hpf != startup.accel_hpf) {
        RCLCPP_ERROR(get_logger(), "Accel filter configuration mismatch");
        ok = false;
    }

    return ok;
}

void OSCPIMUNode::on_frame_trampoline(const oscp_frame_t *f, void *ctx) {
    static_cast<OSCPIMUNode*>(ctx)->on_frame(f);
}

void OSCPIMUNode::on_frame(const oscp_frame_t *f) {
    last_frame_time_ = std::chrono::steady_clock::now();  // Update last frame time for watchdog
    switch (f->type) {
        case OSCP_FRAME_STARTUP: {
            const auto &startup = f->content.startup;

            char mark[11];
            memcpy(mark, startup.mark_number, 10);
            mark[10] = '\0';

            std::string mark_str;
            uint32_t unit_num = 0;

            // Local scope for mutex release 
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                startup_info_.mark_number = std::string(mark);
                startup_info_.unit_number = startup.unit_number;

                startup_info_.operating_mode = (startup.header_byte >> 3) & 0x07;

                startup_info_.gyro_range = startup.gyro_dr;
                startup_info_.accel_range = startup.accel_dr;

                startup_info_.gyro_filters = startup.gyro_filters;
                startup_info_.gyro_lpf = startup.gyro_lpf;
                startup_info_.gyro_hpf = startup.gyro_hpf;

                startup_info_.accel_filters = startup.accel_filters;
                startup_info_.accel_lpf = startup.accel_lpf;
                startup_info_.accel_hpf = startup.accel_hpf;

                startup_info_.incl_range = startup.incl_dr;

                startup_received_.store(true);

                mark_str = startup_info_.mark_number;
                unit_num = startup_info_.unit_number;
            }

            RCLCPP_INFO(get_logger(),"Mark: %s Unit: %u", mark_str.c_str(), unit_num);

            break;
        }

        case OSCP_FRAME_RAW: {
            oscp_raw_t raw = oscp_raw(f);         
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_raw_ = raw;
            has_raw_ = true;
            break;
        }

        case OSCP_FRAME_QUATERNION: {
            oscp_quat_t q = oscp_quat(f);
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_quat_ = q;
            has_quat_ = true;
            break;
        }

        case OSCP_FRAME_EULER: {
            oscp_euler_t e = oscp_euler(f);
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_euler_ = e;
            has_euler_ = true;
            break;
        }

        case OSCP_FRAME_ROT_MATRIX: {
            oscp_rot_mat_t rm = oscp_rot_mat(f);
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_rotation_matrix_ = rm;
            has_rotation_matrix_ = true;
            break;
        }
        
        case OSCP_FRAME_GNSS: {
            oscp_gnss_t g = oscp_gnss(f);
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_gnss_ = g;
            has_gnss_ = true;
            break;
        }
        
        default:
            return;
    }

    if(publish_enable_) {
        publish();
    }
}

/* Utility Functions */
uint32_t OSCPIMUNode::float_to_u32(float value) {
    uint32_t bits;

    static_assert(sizeof(float) == sizeof(uint32_t));

    memcpy(&bits, &value, sizeof(float));

    return bits;
}

double OSCPIMUNode::normalize_degrees(double angle) {
    while(angle > 180.0)
        angle -= 360.0;

    while(angle < -180.0)
        angle += 360.0;

    return angle;
}

/* Publishers Functions */
void OSCPIMUNode::publish() {
    // Snapshot frame flags and counters under lock to avoid races with parser thread
    bool has_raw_local, has_quat_local, has_euler_local, has_rot_local, has_gnss_local;
    uint32_t raw_counter_local, quat_counter_local, euler_counter_local, rot_counter_local, gnss_counter_local;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        has_raw_local = has_raw_;
        has_quat_local = has_quat_;
        has_euler_local = has_euler_;
        has_rot_local = has_rotation_matrix_;
        has_gnss_local = has_gnss_;

        raw_counter_local = latest_raw_.counter;
        quat_counter_local = latest_quat_.counter;
        euler_counter_local = latest_euler_.counter;
        rot_counter_local = latest_rotation_matrix_.counter;
        gnss_counter_local = latest_gnss_.counter;
    }

    if (config_.publish_oscp_raw && has_raw_local && (raw_counter_local != last_raw_counter_)) {
        publish_raw_frame();
        last_raw_counter_ = raw_counter_local;
    }

    if (config_.publish_oscp_quat && has_quat_local && (quat_counter_local != last_quat_counter_)) {
        publish_quat_frame();
        last_quat_counter_ = quat_counter_local;
    }

    if (config_.publish_oscp_euler && has_euler_local && (euler_counter_local != last_euler_counter_)) {
        publish_euler_frame();
        last_euler_counter_ = euler_counter_local;
    }

    if (config_.publish_oscp_rot && has_rot_local && (rot_counter_local != last_rot_counter_)) {
        publish_rot_frame();
        last_rot_counter_ = rot_counter_local;
    }

    if (config_.publish_oscp_gnss && has_gnss_local && (gnss_counter_local != last_gnss_counter_)) {
        publish_gnss_frame();
        last_gnss_counter_ = gnss_counter_local;
    }

    if (config_.publish_standard_ros && has_raw_local && has_quat_local && (raw_counter_local != last_ros_counter_)) {
        publish_imu();
        publish_magnetometer();
        publish_temperature();
        last_ros_counter_ = raw_counter_local;
    }
}

void OSCPIMUNode::publish_raw_frame() {
    // Copy latest raw frame under lock to avoid races with parser thread
    oscp_raw_t raw;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        raw = latest_raw_;
    }

    auto raw_msg = oscp_msgs::msg::OscpRaw();

    raw_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(raw.timestamp_ms) * 1000000ULL);
    raw_msg.header.frame_id = config_.frame_id;

    raw_msg.header_byte = raw.header_byte;
    raw_msg.counter = raw.counter;
    raw_msg.timestamp_ms = raw.timestamp_ms;

    raw_msg.gyro_x = raw.gyro_x;
    raw_msg.gyro_y = raw.gyro_y;
    raw_msg.gyro_z = raw.gyro_z;

    raw_msg.accel_x = raw.accel_x;
    raw_msg.accel_y = raw.accel_y;
    raw_msg.accel_z = raw.accel_z;

    raw_msg.incl_x = raw.incl_x;
    raw_msg.incl_y = raw.incl_y;

    if (mag_calibrating_.load()) {
        std::lock_guard<std::mutex> lock(mag_calibrator_.mag_samples_mutex);
        mag_calibrator_.mag_x_samples.push_back(raw.mag_x);
        mag_calibrator_.mag_y_samples.push_back(raw.mag_y);
        mag_calibrator_.mag_z_samples.push_back(raw.mag_z);
    }

    raw_msg.mag_x = raw.mag_x;
    raw_msg.mag_y = raw.mag_y;
    raw_msg.mag_z = raw.mag_z;

    raw_msg.temp = raw.temp;
    raw_msg.status = raw.status;
    raw_msg.crc = raw.crc;

    raw_frame_pub_->publish(raw_msg);
}

void OSCPIMUNode::publish_quat_frame() {
    oscp_quat_t q_local;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        q_local = latest_quat_;
    }

    auto quat_msg = oscp_msgs::msg::OscpQuat();

    quat_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(q_local.timestamp_ms) * 1000000ULL);
    quat_msg.header.frame_id = config_.frame_id;

    quat_msg.header_byte = q_local.header_byte;
    quat_msg.counter = q_local.counter;
    quat_msg.timestamp_ms = q_local.timestamp_ms;

    // Apply zero orientation offset if set 
    tf2::Quaternion q_current(q_local.x, q_local.y, q_local.z, q_local.w);

    tf2::Quaternion q_corrected = q_current;

    if (zero_orientation_set_) {
        tf2::Quaternion q_zero(
            zero_orientation_offset_.x,
            zero_orientation_offset_.y,
            zero_orientation_offset_.z,
            zero_orientation_offset_.w
        );

        q_corrected = q_zero.inverse() * q_current;
        q_corrected.normalize();
    }

    quat_msg.quat_x = q_corrected.x();
    quat_msg.quat_y = q_corrected.y();
    quat_msg.quat_z = q_corrected.z();
    quat_msg.quat_w = q_corrected.w();

    quat_msg.status = q_local.status;
    quat_msg.crc = q_local.crc;

    quat_frame_pub_->publish(quat_msg);
}


void OSCPIMUNode::publish_euler_frame() {
    oscp_euler_t e_local;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        e_local = latest_euler_;
    }

    auto euler_msg = oscp_msgs::msg::OscpEuler();

    euler_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(e_local.timestamp_ms) * 1000000ULL);
    euler_msg.header.frame_id = config_.frame_id;

    euler_msg.header_byte = e_local.header_byte;
    euler_msg.counter = e_local.counter;
    euler_msg.timestamp_ms = e_local.timestamp_ms;

    if (euler_zeroed_) {
        euler_msg.roll  = normalize_degrees(e_local.roll  - zero_roll_);
        euler_msg.pitch = normalize_degrees(e_local.pitch - zero_pitch_);
        euler_msg.yaw = normalize_degrees(e_local.yaw - zero_yaw_);
    } else {
        euler_msg.roll  = e_local.roll;
        euler_msg.pitch = e_local.pitch;
        euler_msg.yaw   = e_local.yaw;
    }

    euler_msg.status = e_local.status;
    euler_msg.crc = e_local.crc;

    euler_frame_pub_->publish(euler_msg);
}


void OSCPIMUNode::publish_rot_frame() {
    oscp_rot_mat_t rm_local;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        rm_local = latest_rotation_matrix_;
    }

    auto rot_msg = oscp_msgs::msg::OscpRot();

    rot_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(rm_local.timestamp_ms) * 1000000ULL);
    rot_msg.header.frame_id = config_.frame_id;

    rot_msg.header_byte = rm_local.header_byte;
    rot_msg.counter = rm_local.counter;
    rot_msg.timestamp_ms = rm_local.timestamp_ms;

    float corrected[3][3];
    float rm_arr[3][3];
    memcpy(rm_arr, rm_local.rm, sizeof(rm_arr));

    if (rotation_zeroed_) {
        zero_rotation(rm_arr, corrected);
    } else {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                corrected[i][j] = rm_arr[i][j];
            }
        }
    }

    // Row 0
    rot_msg.rotation_matrix[0] = corrected[0][0];
    rot_msg.rotation_matrix[1] = corrected[0][1];
    rot_msg.rotation_matrix[2] = corrected[0][2];

    // Row 1
    rot_msg.rotation_matrix[3] = corrected[1][0];
    rot_msg.rotation_matrix[4] = corrected[1][1];
    rot_msg.rotation_matrix[5] = corrected[1][2];

    // Row 2
    rot_msg.rotation_matrix[6] = corrected[2][0];
    rot_msg.rotation_matrix[7] = corrected[2][1];
    rot_msg.rotation_matrix[8] = corrected[2][2];

    rot_msg.status = rm_local.status;
    rot_msg.crc = rm_local.crc;

    rot_frame_pub_->publish(rot_msg);
}

void OSCPIMUNode::publish_gnss_frame()  {
    oscp_gnss_t g_local;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        g_local = latest_gnss_;
    }

    auto gnss_msg = oscp_msgs::msg::OscpGnss();

    gnss_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(g_local.timestamp_ms) * 1000000ULL);

    gnss_msg.header.frame_id = config_.frame_id;
    gnss_msg.header_byte = g_local.header_byte;
    gnss_msg.counter = g_local.counter;
    gnss_msg.timestamp_ms = g_local.timestamp_ms;
    gnss_msg.fix_type = g_local.gnss_fix_type;
    gnss_msg.num_satellites = g_local.num_satellites;
    gnss_msg.longitude = g_local.longitude;
    gnss_msg.latitude = g_local.latitude;
    gnss_msg.height_mm = g_local.height;
    gnss_msg.h_accuracy_mm = g_local.horizontal_accuracy;
    gnss_msg.v_accuracy_mm = g_local.vertical_accuracy;
    gnss_msg.vel_north_mms = g_local.velocity_north;
    gnss_msg.vel_east_mms  = g_local.velocity_east;
    gnss_msg.vel_down_mms  = g_local.velocity_down;
    gnss_msg.speed_accuracy_mms = g_local.speed_accuracy;
    gnss_msg.heading_motion = g_local.heading_of_motion;
    gnss_msg.heading_accuracy = g_local.heading_accuracy;
    gnss_msg.position_dop = g_local.pdop;
    gnss_msg.last_correction_age = g_local.lastCorrectionAge;
    gnss_msg.gnss_status = g_local.status;
    gnss_msg.status = g_local.status;
    gnss_msg.crc = g_local.crc;

    gnss_frame_pub_->publish(gnss_msg);
}

void OSCPIMUNode::publish_imu() {
    // copy latest raw and quat under lock
    oscp_raw_t raw_local;
    oscp_quat_t quat_local;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        raw_local = latest_raw_;
        quat_local = latest_quat_;
    }

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = config_.frame_id;

    // Angular velocity (°/sec -> rad/sec)
    imu_msg.angular_velocity.x = raw_local.gyro_x * M_PI / 180.0;
    imu_msg.angular_velocity.y = raw_local.gyro_y * M_PI / 180.0;
    imu_msg.angular_velocity.z = raw_local.gyro_z * M_PI / 180.0;

    // Linear acceleration
    if (config_.pub_standard_ros_accel_in_g) {
        imu_msg.linear_acceleration.x = raw_local.accel_x;
        imu_msg.linear_acceleration.y = raw_local.accel_y;
        imu_msg.linear_acceleration.z = raw_local.accel_z;
    } else {
        imu_msg.linear_acceleration.x = raw_local.accel_x * 9.80665;
        imu_msg.linear_acceleration.y = raw_local.accel_y * 9.80665;
        imu_msg.linear_acceleration.z = raw_local.accel_z * 9.80665;
    }

    // Experimentally measured values to be confirmed
    imu_msg.angular_velocity_covariance = {
        3.046e-8, 0.0, 0.0,
        0.0, 3.046e-8, 0.0,
        0.0, 0.0, 3.046e-8
    };

    // Experimentally measured values to be confirmed
    imu_msg.linear_acceleration_covariance = {
        2.164e-8, 0.0, 0.0,
        0.0, 2.164e-8, 0.0,
        0.0, 0.0, 2.164e-8
    };

    tf2::Quaternion q(
        quat_local.x,
        quat_local.y,
        quat_local.z,
        quat_local.w
    );

    if (zero_orientation_set_) {
        tf2::Quaternion q_zero(
            zero_orientation_offset_.x,
            zero_orientation_offset_.y,
            zero_orientation_offset_.z,
            zero_orientation_offset_.w
        );

        q = q_zero.inverse() * q;
    }

    q.normalize();

    imu_msg.orientation.w = q.w();
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();

    // Experimentally measured values to be confirmed
    imu_msg.orientation_covariance = {
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.01
    };

    imu_pub_->publish(imu_msg);
}

void OSCPIMUNode::publish_magnetometer() {
    sensor_msgs::msg::MagneticField mag_msg;

    oscp_raw_t raw_local;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        raw_local = latest_raw_;
    }

    mag_msg.header.stamp = this->now();
    mag_msg.header.frame_id = config_.frame_id;

    // Convert µT to T
    mag_msg.magnetic_field.x = raw_local.mag_x * 1e-6; 
    mag_msg.magnetic_field.y = raw_local.mag_y * 1e-6;
    mag_msg.magnetic_field.z = raw_local.mag_z * 1e-6;

    // Experimentally measured values to be confirmed
    mag_msg.magnetic_field_covariance = {
        3.61e-14, 0.0, 0.0,
        0.0, 3.61e-14, 0.0,
        0.0, 0.0, 3.61e-14
    };

    mag_pub_->publish(mag_msg);
}

void OSCPIMUNode::publish_temperature() {
    sensor_msgs::msg::Temperature temp_msg;
    oscp_raw_t raw_local;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        raw_local = latest_raw_;
    }

    temp_msg.header.stamp = this->now();
    temp_msg.header.frame_id = config_.frame_id;

    temp_msg.temperature = raw_local.temp;
    temp_msg.variance = 0.01;

    temp_pub_->publish(temp_msg);
}

/* Service Functions */
void OSCPIMUNode::get_config_callback(const std::shared_ptr<oscp_imu_ros2::srv::GetIMUConfig::Request> request, std::shared_ptr<oscp_imu_ros2::srv::GetIMUConfig::Response> response) {

    std::string config_text =
        std::string("Transport: ") + oscp_imu::toString(config_.transport) + "\n" +
        "Mode: " + std::to_string(operatingModeToStartup(config_.operating_mode)) + "\n" +
        "Gyro: " + std::to_string(gyroRangeToStartup(config_.gyro_range)) + "\n" +
        "Accel: " + std::to_string(accelRangeToStartup(config_.accel_range)) + "\n" +
        "Incl: " + std::to_string(inclRangeToStartup(config_.incl_range)) + "\n" +
        "Gyro Filter: " + std::to_string(filterModeToStartup(config_.gyro_filter_mode)) + "\n" +
        "Gyro LPF: " + std::to_string(filterCutoffToStartup(config_.gyro_lpf)) + "\n" +
        "Gyro HPF: " + std::to_string(filterCutoffToStartup(config_.gyro_hpf)) + "\n" +
        "Accel Filter: " + std::to_string(filterModeToStartup(config_.accel_filter_mode)) + "\n" +
        "Accel LPF: " + std::to_string(filterCutoffToStartup(config_.accel_lpf)) + "\n" +
        "Accel HPF: " + std::to_string(filterCutoffToStartup(config_.accel_hpf)) + "\n";
    response->config_summary = config_text;
}

bool OSCPIMUNode::stationary_calibrate(double duration) {
    RCLCPP_INFO(get_logger(),"Starting stationary calibration for %.2f seconds",duration);

    double gx = 0;
    double gy = 0;
    double gz = 0;

    double ax = 0;
    double ay = 0;
    double az = 0;

    double ix = 0;
    double iy = 0;

    int samples = 0;

    auto start = this->now();

    while((this->now() - start).seconds() < duration) {
        // Copy latest raw under lock to avoid races with parser thread
        oscp_raw_t raw_local;
        bool have = false;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (has_raw_) {
                raw_local = latest_raw_;
                have = true;
            }
        }

        if (have) {
            gx += raw_local.gyro_x;
            gy += raw_local.gyro_y;
            gz += raw_local.gyro_z;

            ax += raw_local.accel_x;
            ay += raw_local.accel_y;
            az += raw_local.accel_z;

            ix += raw_local.incl_x;
            iy += raw_local.incl_y;

            samples++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if(samples == 0) {
        RCLCPP_ERROR(
            get_logger(),
            "No IMU samples collected"
        );
        return false;
    }

    float gxb = -(gx / samples);
    float gyb = -(gy / samples);
    float gzb = -(gz / samples);

    float axb = -(ax / samples);
    float ayb = -(ay / samples);
    float azb = 1.0f - (az / samples);

    float ixb = -(ix / samples);
    float iyb = -(iy / samples);

    RCLCPP_INFO(get_logger(),"Biases: G %.5f %.5f %.5f A %.5f %.5f %.5f I %.5f %.5f",gxb,gyb,gzb,axb,ayb,azb,ixb,iyb);

    write_float_register(OSCP_USR_REG_GXB, gxb);
    write_float_register(OSCP_USR_REG_GYB, gyb);
    write_float_register(OSCP_USR_REG_GZB, gzb);

    write_float_register(OSCP_USR_REG_AXB, axb);
    write_float_register(OSCP_USR_REG_AYB, ayb);
    write_float_register(OSCP_USR_REG_AZB, azb);

    write_float_register(OSCP_USR_REG_IXB, ixb);
    write_float_register(OSCP_USR_REG_IYB, iyb);

    uint8_t cmd[256];
    size_t cmd_len = 0;

    // Save calibration
    oscp_cmd_save(cmd, sizeof(cmd), &cmd_len, to_oscp(config_.transport));

    write(fd_, cmd, cmd_len);
    tcdrain(fd_);

    // Force Unzeroed State
    zero_orientation_set_ = false;
    euler_zeroed_ = false;
    rotation_zeroed_ = false;

    return true;
}

void OSCPIMUNode::stationary_calibrate_callback(const std::shared_ptr<oscp_imu_ros2::srv::StationaryCalibrate::Request> request,
    std::shared_ptr<oscp_imu_ros2::srv::StationaryCalibrate::Response> response) {
    if (stationary_calibration_thread_.joinable()) {
        if (stationary_calibration_active_.load()) {
            response->success = false;
            response->message = "Stationary calibration already in progress";
            return;
        }
        stationary_calibration_thread_.join();
    }

    stationary_calibration_active_.store(true);
    stationary_calibration_thread_ = std::thread(
        &OSCPIMUNode::stationary_calibration_worker, this, request->duration);

    response->success = true;
    response->message = "Stationary calibration started";
}

void OSCPIMUNode::zero_rotation(float current[3][3], float corrected[3][3]) {
    float result[3][3];

    // corrected = zero^T * current
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            result[i][j] = 0.0f;
            for(int k = 0; k < 3; k++) {
                // transpose zero matrix here
                result[i][j] += zero_rotation_matrix_.rm[k][i] * current[k][j];
            }
        }
    }

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            corrected[i][j] = result[i][j];
        }
    }
}

void OSCPIMUNode::stationary_calibration_worker(double duration) {
    bool success = stationary_calibrate(duration);

    if (success) {
        RCLCPP_INFO(get_logger(), "Stationary calibration completed successfully");
    } else {
        RCLCPP_WARN(get_logger(), "Stationary calibration failed");
    }

    stationary_calibration_active_.store(false);
}

void OSCPIMUNode::zero_orientation_callback(const std::shared_ptr<oscp_imu_ros2::srv::ZeroOrientation::Request> request, std::shared_ptr<oscp_imu_ros2::srv::ZeroOrientation::Response> response) {
    (void)request;

    bool zeroed = false;
    // Snapshot latest frames under lock and apply zeroing based on snapshot
    oscp_quat_t quat_local;
    oscp_euler_t euler_local;
    oscp_rot_mat_t rm_local;
    bool have_quat = false;
    bool have_euler = false;
    bool have_rm = false;

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (has_quat_) {
            quat_local = latest_quat_;
            have_quat = true;
        }
        if (has_euler_) {
            euler_local = latest_euler_;
            have_euler = true;
        }
        if (has_rotation_matrix_) {
            rm_local = latest_rotation_matrix_;
            have_rm = true;
        }
    }

    // Quaternion zero
    if (have_quat) {
        zero_orientation_offset_ = quat_local;
        zero_orientation_set_ = true;
        zeroed = true;
    }

    // Euler zero
    if (have_euler) {
        zero_roll_  = euler_local.roll;
        zero_pitch_ = euler_local.pitch;
        zero_yaw_   = euler_local.yaw;

        euler_zeroed_ = true;
        zeroed = true;
    }

    // Rotation matrix zero
    if (have_rm) {
        zero_rotation_matrix_ = rm_local;

        rotation_zeroed_ = true;
        zeroed = true;
    }

    if(zeroed) {
        response->success = true;
        response->message = "Orientation zeroed successfully";
    }
    else {
        response->success = false;
        response->message = "No orientation data available";
    }
}

/* Action Functions */
void OSCPIMUNode::handle_mag_accept(const std::shared_ptr<GoalHandleMagCal> goal_handle) {
    if (mag_calibration_thread_.joinable()) {
        if (mag_calibrating_.load()) {
            RCLCPP_ERROR(get_logger(), "Magnetometer calibration already active");
            return;
        }
        mag_calibration_thread_.join();
    }

    mag_calibration_thread_ = std::thread(
        &OSCPIMUNode::execute_mag_calibration, this, goal_handle);
}

void OSCPIMUNode::execute_mag_calibration(const std::shared_ptr<GoalHandleMagCal> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Starting magnetometer calibration");

    auto feedback = std::make_shared<MagnetometerCalibration::Feedback>();
    auto action_result = std::make_shared<MagnetometerCalibration::Result>();

    // Clear old samples
    mag_calibrator_.mag_x_samples.clear();
    mag_calibrator_.mag_y_samples.clear();
    mag_calibrator_.mag_z_samples.clear();

    // Enable collection inside publish_raw_frame()
    mag_calibrating_.store(true);

    std::vector<float> mag_x_copy;
    std::vector<float> mag_y_copy;
    std::vector<float> mag_z_copy;

    float gap = 9999.0f;

    while (rclcpp::ok()) {
        // Allow cancel
        if (goal_handle->is_canceling()) {
            mag_calibrating_.store(false);

            action_result->success = false;
            goal_handle->canceled(action_result);

            return;
        }

        {
            std::lock_guard<std::mutex> lock(mag_calibrator_.mag_samples_mutex);
            mag_x_copy = mag_calibrator_.mag_x_samples;
            mag_y_copy = mag_calibrator_.mag_y_samples;
            mag_z_copy = mag_calibrator_.mag_z_samples;
        }

        if (mag_x_copy.size() > 20) {
            auto estimate = mag_calibrator_.estimate_bias(mag_x_copy, mag_y_copy, mag_z_copy);
            gap = mag_calibrator_.compute_gap(mag_x_copy, mag_y_copy, mag_z_copy, estimate.offset_x, estimate.offset_y, estimate.offset_z);
        }

        feedback->progress = gap;
        feedback->status = "collecting";
        goal_handle->publish_feedback(feedback);

        // Stop once the user supplied threshold is reached
        if (gap <= mag_calibrator_.target_gap) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop collecting samples
    mag_calibrating_.store(false);

    {
        std::lock_guard<std::mutex> lock(mag_calibrator_.mag_samples_mutex);
        mag_x_copy = mag_calibrator_.mag_x_samples;
        mag_y_copy = mag_calibrator_.mag_y_samples;
        mag_z_copy = mag_calibrator_.mag_z_samples;
    }

    auto sample_count = mag_x_copy.size();
    RCLCPP_INFO(this->get_logger(),"Samples collected: %zu", sample_count);

    // Run final calibration solve on local copies
    auto calibration = mag_calibrator_.solve(mag_x_copy, mag_y_copy, mag_z_copy);

    // Write to registers
    write_float_register(OSCP_USR_REG_MXB, calibration.offset_x);
    write_float_register(OSCP_USR_REG_MYB, calibration.offset_y);
    write_float_register(OSCP_USR_REG_MZB, calibration.offset_z);

    write_float_register(OSCP_USR_REG_MXX, calibration.soft_iron[0][0]);
    write_float_register(OSCP_USR_REG_MYX, calibration.soft_iron[0][1]);
    write_float_register(OSCP_USR_REG_MZX, calibration.soft_iron[0][2]);

    write_float_register(OSCP_USR_REG_MXY, calibration.soft_iron[1][0]);
    write_float_register(OSCP_USR_REG_MYY, calibration.soft_iron[1][1]);
    write_float_register(OSCP_USR_REG_MZY, calibration.soft_iron[1][2]);

    write_float_register(OSCP_USR_REG_MXZ, calibration.soft_iron[2][0]);
    write_float_register(OSCP_USR_REG_MYZ, calibration.soft_iron[2][1]);
    write_float_register(OSCP_USR_REG_MZZ, calibration.soft_iron[2][2]);

    // Save to flash if required
    uint8_t cmd[256];
    size_t cmd_len = 0;

    oscp_cmd_save(cmd, sizeof(cmd), &cmd_len, to_oscp(config_.transport));
    write(fd_, cmd, cmd_len);
    tcdrain(fd_);

    //Copy calibration result into ROS action result
    action_result->success = true;
    action_result->hard_iron_x = calibration.offset_x;
    action_result->hard_iron_y = calibration.offset_y;
    action_result->hard_iron_z = calibration.offset_z;
    action_result->fit_error = calibration.fit_error;
    goal_handle->succeed(action_result);

    RCLCPP_INFO(this->get_logger(), "Magnetometer calibration finished");
}

rclcpp_action::GoalResponse OSCPIMUNode::handle_mag_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MagnetometerCalibration::Goal> goal) {
    (void)goal;

    if (mag_calibrating_.load()) {
        RCLCPP_WARN(get_logger(), "Magnetometer calibration already running");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (mag_calibration_thread_.joinable() && !mag_calibrating_.load()) {
        mag_calibration_thread_.join();
    }

    mag_calibrator_.target_gap = 1.0f;
    RCLCPP_INFO(get_logger(), "Starting mag calibration");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OSCPIMUNode::handle_mag_cancel(const std::shared_ptr<GoalHandleMagCal>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

/* Main Entry Point */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OSCPIMUNode>());
    rclcpp::shutdown();
    return 0;
}
