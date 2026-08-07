#include <rclcpp/rclcpp.hpp>

#include <oscp_msgs/msg/oscp_raw.hpp>

#include <motion_cal_wrapper/msg/mag_calibration.hpp>

#include <mutex>
#include <array>


extern "C"
{
#include "motion_cal_wrapper/magcal.h"
}

// Wrapper declarations from motioncal_shim.c

extern "C" {

void motioncal_reset(void);
int motioncal_add_sample(int16_t rawx, int16_t rawy, int16_t rawz);
void motioncal_refresh_quality(void);
float motioncal_gap_error(void);
float motioncal_variance_error(void);
float motioncal_wobble_error(void);
float motioncal_spherical_fit_error(void);
float motioncal_fit_error(void);
float motioncal_field_strength(void);

int motioncal_valid_cal(void);

void motioncal_get_offset(float out[3]);
void motioncal_get_matrix(float out[9]);

int motioncal_sample_count(void);
}

class MotionCalNode : public rclcpp::Node {

public:

    MotionCalNode(): Node("motion_cal_node") {

        RCLCPP_INFO(get_logger(),"Starting MotionCal node");
        motioncal_reset();

        raw_sub_ = create_subscription<oscp_msgs::msg::OscpRaw>("/oscp/raw",50,std::bind(&MotionCalNode::raw_callback,this,std::placeholders::_1));
        calibration_pub_ = create_publisher<motion_cal_wrapper::msg::MagCalibration>("/oscp/mag_calibration",10);
        timer_ = create_wall_timer(std::chrono::milliseconds(500),std::bind(&MotionCalNode::publish_status,this));
    }

private:

    void raw_callback(const oscp_msgs::msg::OscpRaw::SharedPtr msg) {
        int accepted = motioncal_add_sample(msg->mag_x, msg->mag_y, msg->mag_z);

        if(accepted) {
            RCLCPP_INFO(get_logger(), "New mag calibration accepted");
        }
    }

    void publish_status() {
        motioncal_refresh_quality();

        auto msg = motion_cal_wrapper::msg::MagCalibration();
        
        float offset[3];
        float matrix[9];

        motioncal_get_offset(offset);
        motioncal_get_matrix(matrix);

        // Hard iron bias
        msg.bias_x = offset[0];
        msg.bias_y = offset[1];
        msg.bias_z = offset[2];

        // Soft iron correction matrix
        for(int i = 0; i < 9; i++) {
            msg.soft_matrix[i] = matrix[i];
        }

        // Calibration quality
        msg.field_strength = motioncal_field_strength();

        msg.fit_error = motioncal_fit_error();

        msg.gap_error = motioncal_gap_error();

        // Status
        msg.solver = motioncal_valid_cal();

        msg.samples_used = motioncal_sample_count();

        calibration_pub_->publish(msg);
    }

    rclcpp::Subscription<oscp_msgs::msg::OscpRaw>::SharedPtr raw_sub_;
    rclcpp::Publisher<motion_cal_wrapper::msg::MagCalibration>::SharedPtr calibration_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MotionCalNode>());
    rclcpp::shutdown();
    return 0;
}