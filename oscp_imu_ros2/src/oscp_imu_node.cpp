#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <oscp_msgs/msg/oscp_raw.hpp>
#include <oscp_msgs/msg/oscp_rot.hpp>
#include <oscp_msgs/msg/oscp_quat.hpp>
#include <oscp_msgs/msg/oscp_euler.hpp>
#include <oscp_msgs/msg/oscp_gnss.hpp>

#include "oscp_imu_ros2/oscp_imu_config.hpp"
#include "oscp_imu_ros2/oscp_imu_param_translator.hpp"
#include "oscp_imu_ros2/oscp_imu_rosparam.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

extern "C" {
#include "oscp_imu.h"
}

class OSCPIMUNode : public rclcpp::Node {
public:
    OSCPIMUNode() : Node("oscp_imu_node") {        
        // Default values for initialization that wont work without being set by the user
        declare_parameter("device", std::string(""));
        declare_parameter("baudrate", -1);
        declare_parameter("frame_id", std::string(""));
        declare_parameter("pub_accel_in_g", false);
        declare_parameter("transport",imu::toString(config_.transport));

        declare_parameter("operating_mode", imu::toString(config_.operating_mode));

        declare_parameter("gyro_range", imu::toString(config_.gyro_range));
        declare_parameter("accel_range", imu::toString(config_.accel_range));
        declare_parameter("incl_range", imu::toString(config_.incl_range));

        declare_parameter("gyro_filter_mode", imu::toString(config_.gyro_filter_mode));
        declare_parameter("gyro_lpf", imu::toString(config_.gyro_lpf));
        declare_parameter("gyro_hpf", imu::toString(config_.gyro_hpf));

        declare_parameter("misalignment", config_.misalignment);

        declare_parameter("accel_filter_mode", imu::toString(config_.accel_filter_mode));
        declare_parameter("accel_lpf", imu::toString(config_.accel_lpf));
        declare_parameter("accel_hpf", imu::toString(config_.accel_hpf));

        declare_parameter("ahrs_convention", imu::toString(config_.ahrs_convention));
        declare_parameter("ahrs_heading_source", imu::toString(config_.ahrs_heading));
        
        declare_parameter("standard_ros_enabled", config_.enable_ros);
        declare_parameter("oscp_raw_enabled", config_.enable_raw);
        declare_parameter("oscp_euler_enabled", config_.enable_euler);
        declare_parameter("oscp_quat_enabled", config_.enable_quat);
        declare_parameter("oscp_rotation_matrix_enabled", config_.enable_rot);
        declare_parameter("oscp_gnss_enabled", config_.enable_gnss);

        device_ = this->get_parameter("device").as_string();
        baudrate_ = static_cast<int>(this->get_parameter("baudrate").as_int());        
        frame_id_ = this->get_parameter("frame_id").as_string();
        pub_accel_in_g_ = this->get_parameter("pub_accel_in_g").as_bool();

        if (device_.empty() || baudrate_ < 0 || frame_id_.empty()) {
                RCLCPP_FATAL(get_logger(), "Required parameters not set — set in launch file");
                throw std::runtime_error("Missing required parameters");
            }

        config_.operating_mode = imu::parseOperatingMode(get_parameter("operating_mode").as_string());
        config_.transport = imu::parseTransport(get_parameter("transport").as_string());
        config_.gyro_range = imu::parseGyroRange(get_parameter("gyro_range").as_string());
        config_.accel_range = imu::parseAccelRange(get_parameter("accel_range").as_string());
        config_.incl_range = imu::parseInclRange(get_parameter("incl_range").as_string());
        config_.misalignment = get_parameter("misalignment").as_bool();
        config_.gyro_filter_mode = imu::parseFilterMode(get_parameter("gyro_filter_mode").as_string());
        config_.gyro_lpf = imu::parseFilterCutoff(get_parameter("gyro_lpf").as_string());
        config_.gyro_hpf = imu::parseFilterCutoff(get_parameter("gyro_hpf").as_string());
        config_.accel_filter_mode = imu::parseFilterMode(get_parameter("accel_filter_mode").as_string());
        config_.accel_lpf = imu::parseFilterCutoff(get_parameter("accel_lpf").as_string());
        config_.accel_hpf = imu::parseFilterCutoff(get_parameter("accel_hpf").as_string());
        config_.ahrs_convention = imu::parseAHRSConvention(get_parameter("ahrs_convention").as_string());
        config_.ahrs_heading = imu::parseAHRSHeadingSource(get_parameter("ahrs_heading_source").as_string());
        transport_ = toOSCP(config_.transport);

        config_.enable_ros = this->get_parameter("standard_ros_enabled").as_bool();
        config_.enable_raw = this->get_parameter("oscp_raw_enabled").as_bool();
        config_.enable_euler = this->get_parameter("oscp_euler_enabled").as_bool();
        config_.enable_quat = this->get_parameter("oscp_quat_enabled").as_bool();
        config_.enable_rot = this->get_parameter("oscp_rotation_matrix_enabled").as_bool(); 
        config_.enable_gnss = this->get_parameter("oscp_gnss_enabled").as_bool();

        // Standard ROS publishers
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("oscp/imu/data", 20);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("oscp/imu/mag", 20);
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("oscp/imu/temp", 20);

        // OSCP frame publishers
        raw_frame_pub_  = create_publisher<oscp_msgs::msg::OscpRaw>("oscp/raw", 20);
        quat_frame_pub_ = create_publisher<oscp_msgs::msg::OscpQuat>("oscp/quat", 20);
        euler_frame_pub_= create_publisher<oscp_msgs::msg::OscpEuler>("oscp/euler", 20);
        rot_frame_pub_  = create_publisher<oscp_msgs::msg::OscpRot>("oscp/rot", 20);
        gnss_frame_pub_ = create_publisher<oscp_msgs::msg::OscpGnss>("oscp/gnss", 20);

        oscp_parser_init(&parser_, &OSCPIMUNode::on_frame_trampoline, this);

        fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", device_.c_str());
            return;
        }

        if(config_.transport == imu::Transport::RS422)
        {
            configure_rs422(fd_);
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(),
                "CANFD transport selected but CAN initialization is not implemented"
            );
            return;
        }
        configure_imu();

        timer_ = create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&OSCPIMUNode::read_transport, this)
        );

        RCLCPP_INFO(get_logger(), "IMU node started on %s", device_.c_str());
        RCLCPP_INFO(get_logger(),"PARAMS: device=%s baudrate=%d frame_id=%s",device_.c_str(),baudrate_,frame_id_.c_str());
    }

    ~OSCPIMUNode() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    imu::IMUConfig config_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpRaw>::SharedPtr raw_frame_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpQuat>::SharedPtr quat_frame_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpEuler>::SharedPtr euler_frame_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpRot>::SharedPtr rot_frame_pub_;
    rclcpp::Publisher<oscp_msgs::msg::OscpGnss>::SharedPtr gnss_frame_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int fd_{-1};

    bool pub_accel_in_g_;

    std::string device_;
    int baudrate_;
    std::string frame_id_;
    oscp_transport_t transport_;

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

    uint8_t buf_[2048];

    speed_t baud_to_termios(int baud) {
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

    void configure_rs422(int fd) {
        struct termios tty{};
        tcgetattr(fd, &tty);

        speed_t spd = baud_to_termios(baudrate_);

        cfsetospeed(&tty, spd);
        cfsetispeed(&tty, spd);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        tcsetattr(fd, TCSANOW, &tty);

        RCLCPP_INFO(get_logger(), "Serial configured: %d baud, 8N1", baudrate_);
    }

   void configure_imu()
        {
            uint8_t cmd[256];
            size_t cmd_len = 0;

            auto send_command = [&](oscp_err_t result)
            {
                if (result != OSCP_OK)
                {
                    RCLCPP_ERROR(get_logger(),"Failed to generate IMU command");
                    return;
                }

                ssize_t written = write(fd_, cmd, cmd_len);

                if (written < 0)
                {
                    RCLCPP_ERROR(get_logger(),"Failed to write IMU command");
                    return;
                }

                tcdrain(fd_);

                usleep(100000);
            };


            //Enter configuration mode
            send_command(oscp_cmd_config(cmd,sizeof(cmd),&cmd_len,transport_));
        
            send_command(oscp_cmd_dri(cmd,sizeof(cmd),&cmd_len,toOSCP(config_.incl_range),transport_));
            send_command(oscp_cmd_disable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_RAW, transport_));
            send_command(oscp_cmd_disable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_EULER, transport_));
            send_command(oscp_cmd_disable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_QUATERNION, transport_));
            send_command(oscp_cmd_disable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_ROT_MATRIX, transport_));

            send_command(oscp_cmd_om(cmd, sizeof(cmd), &cmd_len, toOSCP(config_.operating_mode), transport_));
            send_command(oscp_cmd_drg(cmd, sizeof(cmd), &cmd_len, toOSCP(config_.gyro_range), transport_));
            send_command(oscp_cmd_dra(cmd, sizeof(cmd), &cmd_len, toOSCP(config_.accel_range), transport_));
            send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len, OSCP_USR_REG_FCO, toOSCP(config_.ahrs_convention), transport_));
            send_command(oscp_cmd_wr(cmd, sizeof(cmd), &cmd_len, OSCP_USR_REG_FHS, toOSCP(config_.ahrs_heading), transport_));

            if(config_.enable_raw){send_command(oscp_cmd_enable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_RAW, transport_));}
            if(config_.enable_euler){send_command(oscp_cmd_enable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_EULER, transport_));}
            if(config_.enable_quat){send_command(oscp_cmd_enable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_QUATERNION, transport_));}
            if(config_.enable_rot){send_command(oscp_cmd_enable_oft(cmd, sizeof(cmd), &cmd_len, OSCP_FRAME_SEL_ROT_MATRIX, transport_));}

            send_command(oscp_cmd_save(cmd,sizeof(cmd),&cmd_len,transport_));
            send_command(oscp_cmd_exit(cmd, sizeof(cmd), &cmd_len, transport_));

            RCLCPP_INFO(get_logger(), "IMU configuration applied successfully");
        }

    void read_transport() {
        int n = read(fd_, buf_, sizeof(buf_));

        if (n > 0) {
            oscp_parser_feed_buf(&parser_, buf_, (size_t)n);
        }
    }

    static void on_frame_trampoline(const oscp_frame_t *f, void *ctx) {
        static_cast<OSCPIMUNode*>(ctx)->on_frame(f);
    }

    void on_frame(const oscp_frame_t *f) {
        switch (f->type) {
            case OSCP_FRAME_RAW:
                latest_raw_ = oscp_raw(f);
                has_raw_ = true;
                break;

            case OSCP_FRAME_QUATERNION:
                latest_quat_ = oscp_quat(f);
                has_quat_ = true;
                break;

            case OSCP_FRAME_EULER:
                latest_euler_ = oscp_euler(f);
                has_euler_ = true;
                break;

            case OSCP_FRAME_ROT_MATRIX:
                latest_rotation_matrix_ = oscp_rot_mat(f);
                has_rotation_matrix_ = true;
                break;
            
            case OSCP_FRAME_GNSS:
                latest_gnss_ = oscp_gnss(f);
                has_gnss_ = true;
                break;
            
            default:
                return;
        }

        publish();
    }

    void publish() {
        if (!has_raw_) return;

        if (config_.enable_raw)                            publish_raw_frame();
        if (config_.enable_quat  && has_quat_)             publish_quat_frame();
        if (config_.enable_euler && has_euler_)            publish_euler_frame();
        if (config_.enable_rot   && has_rotation_matrix_) publish_rot_frame();
        if (config_.enable_gnss && has_gnss_) publish_gnss_frame();

        if (config_.enable_ros) {
            publish_imu();
            publish_magnetometer();
            publish_temperature();
        }
    }

    void publish_raw_frame() {
        auto raw_msg = oscp_msgs::msg::OscpRaw();

        raw_msg.header.stamp = rclcpp::Time(
            static_cast<int64_t>(latest_raw_.timestamp_ms) * 1000000ULL
        );
        raw_msg.header.frame_id = frame_id_;

        raw_msg.header_byte = latest_raw_.header_byte;
        raw_msg.counter = latest_raw_.counter;
        raw_msg.timestamp_ms = latest_raw_.timestamp_ms;

        raw_msg.gyro_x = latest_raw_.gyro_x;
        raw_msg.gyro_y = latest_raw_.gyro_y;
        raw_msg.gyro_z = latest_raw_.gyro_z;

        raw_msg.accel_x = latest_raw_.accel_x;
        raw_msg.accel_y = latest_raw_.accel_y;
        raw_msg.accel_z = latest_raw_.accel_z;

        raw_msg.incl_x = latest_raw_.incl_x;
        raw_msg.incl_y = latest_raw_.incl_y;

        raw_msg.mag_x = latest_raw_.mag_x;
        raw_msg.mag_y = latest_raw_.mag_y;
        raw_msg.mag_z = latest_raw_.mag_z;

        raw_msg.temp = latest_raw_.temp;
        raw_msg.status = latest_raw_.status;
        raw_msg.crc = latest_raw_.crc;

        raw_frame_pub_->publish(raw_msg);
    }

    void publish_quat_frame() {
        auto quat_msg = oscp_msgs::msg::OscpQuat();

        quat_msg.header.stamp = rclcpp::Time(
            static_cast<int64_t>(latest_quat_.timestamp_ms) * 1000000ULL
        );
        quat_msg.header.frame_id = frame_id_;

        quat_msg.header_byte = latest_quat_.header_byte;
        quat_msg.counter = latest_quat_.counter;
        quat_msg.timestamp_ms = latest_quat_.timestamp_ms;

        quat_msg.quat_w = latest_quat_.w;
        quat_msg.quat_x = latest_quat_.x;
        quat_msg.quat_y = latest_quat_.y;
        quat_msg.quat_z = latest_quat_.z;

        quat_msg.status = latest_quat_.status;
        quat_msg.crc = latest_quat_.crc;

        quat_frame_pub_->publish(quat_msg);
    }


    void publish_euler_frame() {
        auto euler_msg = oscp_msgs::msg::OscpEuler();

        euler_msg.header.stamp = rclcpp::Time(
            static_cast<int64_t>(latest_euler_.timestamp_ms) * 1000000ULL
        );
        euler_msg.header.frame_id = frame_id_;

        euler_msg.header_byte = latest_euler_.header_byte;
        euler_msg.counter = latest_euler_.counter;
        euler_msg.timestamp_ms = latest_euler_.timestamp_ms;

        euler_msg.roll = latest_euler_.roll;
        euler_msg.pitch = latest_euler_.pitch;
        euler_msg.yaw = latest_euler_.yaw;
        
        euler_msg.status = latest_euler_.status;
        euler_msg.crc = latest_euler_.crc;

        euler_frame_pub_->publish(euler_msg);
    }


    void publish_rot_frame() {
        auto rot_msg = oscp_msgs::msg::OscpRot();

        rot_msg.header.stamp = rclcpp::Time(
            static_cast<int64_t>(latest_rotation_matrix_.timestamp_ms) * 1000000ULL
        );
        rot_msg.header.frame_id = frame_id_;

        rot_msg.header_byte = latest_rotation_matrix_.header_byte;
        rot_msg.counter = latest_rotation_matrix_.counter;
        rot_msg.timestamp_ms = latest_rotation_matrix_.timestamp_ms;

        // Row 0
        rot_msg.rotation_matrix[0] = latest_rotation_matrix_.rm[0][0];
        rot_msg.rotation_matrix[1] = latest_rotation_matrix_.rm[0][1];
        rot_msg.rotation_matrix[2] = latest_rotation_matrix_.rm[0][2];
    
        // Row 1
        rot_msg.rotation_matrix[3] = latest_rotation_matrix_.rm[1][0];
        rot_msg.rotation_matrix[4] = latest_rotation_matrix_.rm[1][1];
        rot_msg.rotation_matrix[5] = latest_rotation_matrix_.rm[1][2];
    
        // Row 2
        rot_msg.rotation_matrix[6] = latest_rotation_matrix_.rm[2][0];
        rot_msg.rotation_matrix[7] = latest_rotation_matrix_.rm[2][1];
        rot_msg.rotation_matrix[8] = latest_rotation_matrix_.rm[2][2];

        rot_msg.status = latest_rotation_matrix_.status;
        rot_msg.crc = latest_rotation_matrix_.crc;

        rot_frame_pub_->publish(rot_msg);
    }

    void publish_gnss_frame() 
    {
        auto gnss_msg = oscp_msgs::msg::OscpGnss();

        gnss_msg.header.stamp = rclcpp::Time(
            static_cast<int64_t>(latest_gnss_.timestamp_ms) * 1000000ULL
        );

        gnss_msg.header.frame_id = frame_id_;
        gnss_msg.header_byte = latest_gnss_.header_byte;
        gnss_msg.counter = latest_gnss_.counter;
        gnss_msg.timestamp_ms = latest_gnss_.timestamp_ms;
        gnss_msg.fix_type = latest_gnss_.gnss_fix_type;
        gnss_msg.num_satellites = latest_gnss_.num_satellites;
        gnss_msg.longitude = latest_gnss_.longitude;
        gnss_msg.latitude = latest_gnss_.latitude;
        gnss_msg.height_mm = latest_gnss_.height;
        gnss_msg.h_accuracy_mm = latest_gnss_.horizontal_accuracy;
        gnss_msg.v_accuracy_mm = latest_gnss_.vertical_accuracy;
        gnss_msg.vel_north_mms = latest_gnss_.velocity_north;
        gnss_msg.vel_east_mms  = latest_gnss_.velocity_east;
        gnss_msg.vel_down_mms  = latest_gnss_.velocity_down;
        gnss_msg.speed_accuracy_mms = latest_gnss_.speed_accuracy;
        gnss_msg.heading_motion = latest_gnss_.heading_of_motion;
        gnss_msg.heading_accuracy = latest_gnss_.heading_accuracy;
        gnss_msg.position_dop = latest_gnss_.pdop;
        gnss_msg.last_correction_age = latest_gnss_.lastCorrectionAge;
        gnss_msg.gnss_status = latest_gnss_.status;
        gnss_msg.status = latest_gnss_.status;
        gnss_msg.crc = latest_gnss_.crc;


        gnss_frame_pub_->publish(gnss_msg);
    }

    void publish_imu() {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = frame_id_;

        // Angular velocity (°/sec -> rad/sec)
        imu_msg.angular_velocity.x = latest_raw_.gyro_x * M_PI / 180.0;
        imu_msg.angular_velocity.y = latest_raw_.gyro_y * M_PI / 180.0;
        imu_msg.angular_velocity.z = latest_raw_.gyro_z * M_PI / 180.0;

        // Linear acceleration
        if (pub_accel_in_g_) {
            imu_msg.linear_acceleration.x = latest_raw_.accel_x;
            imu_msg.linear_acceleration.y = latest_raw_.accel_y;
            imu_msg.linear_acceleration.z = latest_raw_.accel_z;
        } else {
            imu_msg.linear_acceleration.x = latest_raw_.accel_x * 9.80665;
            imu_msg.linear_acceleration.y = latest_raw_.accel_y * 9.80665;
            imu_msg.linear_acceleration.z = latest_raw_.accel_z * 9.80665;
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

        if (has_quat_) {
            imu_msg.orientation.w = latest_quat_.w;
            imu_msg.orientation.x = latest_quat_.x;
            imu_msg.orientation.y = latest_quat_.y;
            imu_msg.orientation.z = latest_quat_.z;

            // Experimentally measured values to be confirmed
            imu_msg.orientation_covariance = {
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            };
        } else {
            imu_msg.orientation.w = 1.0;
            
            // Experimentally measured values to be confirmed
            imu_msg.orientation_covariance = {
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            };
        }

        imu_pub_->publish(imu_msg);
    }

    void publish_magnetometer() {
        sensor_msgs::msg::MagneticField mag_msg;

        mag_msg.header.stamp = this->now();
        mag_msg.header.frame_id = frame_id_;

        // Convert µT to T
        mag_msg.magnetic_field.x = latest_raw_.mag_x * 1e-6; 
        mag_msg.magnetic_field.y = latest_raw_.mag_y * 1e-6;
        mag_msg.magnetic_field.z = latest_raw_.mag_z * 1e-6;

        // Experimentally measured values to be confirmed
        mag_msg.magnetic_field_covariance = {
            3.61e-14, 0.0, 0.0,
            0.0, 3.61e-14, 0.0,
            0.0, 0.0, 3.61e-14
        };

        mag_pub_->publish(mag_msg);
    }

    void publish_temperature() {
        sensor_msgs::msg::Temperature temp_msg;
        temp_msg.header.stamp = this->now();
        temp_msg.header.frame_id = frame_id_;

        temp_msg.temperature = latest_raw_.temp;
        temp_msg.variance = 0.01;

        temp_pub_->publish(temp_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OSCPIMUNode>());
    rclcpp::shutdown();
    return 0;
}