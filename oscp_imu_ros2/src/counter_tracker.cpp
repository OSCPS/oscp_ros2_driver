#include <cstdint>
#include <fstream>
#include <functional>
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "oscp_msgs/msg/oscp_raw.hpp"
#include "oscp_msgs/msg/oscp_quat.hpp"
#include "oscp_msgs/msg/oscp_euler.hpp"
#include "oscp_msgs/msg/oscp_rot.hpp"
#include "oscp_msgs/msg/oscp_gnss.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

class CounterMonitor : public rclcpp::Node
{
public:
    CounterMonitor() : Node("counter_monitor") {
        // Time argument (seconds)
        this->declare_parameter<double>("duration", 0.0);
        duration_ = this->get_parameter("duration").as_double();

        csv_.open("counter_gaps.csv");
        csv_ << "timestamp_ms,last_counter,expected_counter,received_counter,missed\n";

        rclcpp::QoS high_rate_qos = rclcpp::SensorDataQoS();
        high_rate_qos.keep_last(1000);

        sub_ = create_subscription<oscp_msgs::msg::OscpRaw>("/oscp/raw", high_rate_qos, std::bind(&CounterMonitor::raw_callback, this, std::placeholders::_1));
        topic_counts_["/oscp/raw"] = 0;

        create_topic_subscription<oscp_msgs::msg::OscpQuat>("/oscp/quat", high_rate_qos);
        create_topic_subscription<oscp_msgs::msg::OscpEuler>("/oscp/euler", high_rate_qos);
        create_topic_subscription<oscp_msgs::msg::OscpRot>("/oscp/rot", high_rate_qos);
        create_topic_subscription<oscp_msgs::msg::OscpGnss>("/oscp/gnss", high_rate_qos);
        create_topic_subscription<sensor_msgs::msg::Imu>("/oscp/imu/data", high_rate_qos);
        create_topic_subscription<sensor_msgs::msg::MagneticField>("/oscp/imu/mag", high_rate_qos);
        create_topic_subscription<sensor_msgs::msg::Temperature>("/oscp/imu/temp", high_rate_qos);

        hz_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CounterMonitor::hz_callback, this));

        if (duration_ > 0.0) {
            timer_ = create_wall_timer(
                std::chrono::duration<double>(duration_),
                std::bind(&CounterMonitor::stop, this));

            RCLCPP_INFO(
                get_logger(),
                "Counter monitor started for %.2f seconds.",
                duration_);
        }
        else {
            RCLCPP_INFO(
                get_logger(),
                "Counter monitor started (no time limit).");
        }
    }

    ~CounterMonitor() 
    {
        if (csv_.is_open())
            csv_.close();

        RCLCPP_INFO(
            get_logger(),
            "Finished. Total packets received: %lu, Total missed packets: %lu",
            total_counter_,
            total_missed_);
    }

private:
    void raw_callback(const oscp_msgs::msg::OscpRaw::SharedPtr msg) {
        count_topic_message("/oscp/raw");

        uint8_t current = msg->counter;
        uint32_t timestamp_ms = msg->timestamp_ms;

        if (!received_first_)
        {
            last_counter_ = current;
            received_first_ = true;
            return;
        }

        // Handles 255 -> 0 automatically
        uint8_t expected = static_cast<uint8_t>(last_counter_ + 1);

        if (current != expected)
        {
            uint8_t missed = static_cast<uint8_t>(current - expected);
            total_missed_ += missed;

            csv_
                << timestamp_ms << ","
                << static_cast<int>(last_counter_) << ","
                << static_cast<int>(expected) << ","
                << static_cast<int>(current) << ","
                << static_cast<int>(missed)
                << "\n";

            csv_.flush();

            RCLCPP_WARN(
                get_logger(),
                "Gap detected at %u ms: last=%u expected=%u received=%u missed=%u (total=%lu)",
                timestamp_ms,
                last_counter_,
                expected,
                current,
                missed,
                total_missed_);
        }

        last_counter_ = current;
        total_counter_++;
    }

    template<typename MsgT>
    void create_topic_subscription(const std::string &topic_name, const rclcpp::QoS &qos) {
        topic_counts_[topic_name] = 0;
        subs_.push_back(create_subscription<MsgT>(topic_name, qos, [this, topic_name](typename MsgT::SharedPtr) {
            this->count_topic_message(topic_name);
        }));
    }

    void count_topic_message(const std::string &topic_name) {
        topic_counts_[topic_name]++;
    }

    void hz_callback() {
        for (const auto &entry : topic_counts_) {
            RCLCPP_INFO(
                get_logger(),
                "%s: %.1f Hz (%lu msgs)",
                entry.first.c_str(),
                static_cast<double>(entry.second),
                entry.second);
        }

        for (auto &entry : topic_counts_) {
            entry.second = 0;
        }
    }

    void stop() {
        RCLCPP_INFO(
            get_logger(),
            "Time limit reached. Stopping counter monitor.");

        rclcpp::shutdown();
    }

    rclcpp::Subscription<oscp_msgs::msg::OscpRaw>::SharedPtr sub_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr hz_timer_;

    std::ofstream csv_;

    double duration_{0.0};

    bool received_first_{false};
    uint8_t last_counter_{0};
    uint64_t total_missed_{0};
    uint64_t total_counter_{0};

    std::unordered_map<std::string, uint64_t> topic_counts_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CounterMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}