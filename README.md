# OSCP IMU ROS2 Driver (oscp_imu_ros2)

Lightweight, configurable ROS 2 driver for OSCP-family IMUs. The node decodes the device's operating frames and publishes both native OSCP frame messages and standard ROS `sensor_msgs` types.

## Overview

- Supported outputs: raw IMU frames, quaternion, Euler angles, rotation matrix, GNSS (if available), and standard ROS `sensor_msgs` (`Imu`, `MagneticField`, `Temperature`).
- Designed for real-time/high-rate telemetry; uses `SensorDataQoS` for publishers and subscribers by default in code.

## Features

- Multi-format IMU output (native OSCP messages and standard ROS messages)
- ROS 2 native integration (rclcpp, action/service support)
- Configurable runtime parameters via YAML/launch
- Lightweight serial/CAN-FD transport layer

## Installation

Clone into your ROS 2 workspace and build:

```bash
cd ~/ros2_ws/src
git clone https://github.com/OSCPS/oscp_ros2_driver
cd ..
rosdep update
rosdep install --from-paths src -y --ignore-src --rosdistro "$ROS_DISTRO"
colcon build --packages-select oscp_imu_ros2
source install/setup.bash
```

## Usage

Launch the IMU node using the provided launch file and config YAML:

```bash
ros2 launch oscp_imu_ros2 oscp_imu.launch.py config_file:=default.yaml
```

You can override node parameters via the launch file or with `--ros-args -p`.

## Launch / Config Parameters

The node reads parameters from a YAML file passed as `config_file` to the launch. Key parameters include:

- `device` (string): serial device (default `/dev/ttyUSB0`)
- `baudrate` (int): serial baud rate (default `921600`)
- `frame_id` (string): TF frame id for published messages (default `imu_link`)
- `operating_mode` (string): e.g. `MEDIUM` (affects internal IMU rates)
- `publish_standard_ros` (bool): enable publishing `sensor_msgs::Imu` + others
- `publish_oscp_raw` (bool): enable native `/oscp/raw` messages
- `publish_oscp_quat` (bool): enable native `/oscp/quat` messages
- `publish_oscp_euler` (bool): enable native `/oscp/euler` messages
- `publish_oscp_rot` (bool): enable native `/oscp/rot` messages
- `publish_oscp_gnss` (bool): enable native `/oscp/gnss` messages
- `watchdog_timeout_ms` (double): timeout for frame watchdog
- `parser_stats_log_interval_s` (double): interval to log parser stats (0 disables)

Refer to `src/oscp_ros2_driver/oscp_imu_ros2/config/default.yaml` for the full default set.

## Topics

Native OSCP frame topics (types are in `oscp_msgs`):

- `/oscp/raw` (`oscp_msgs::msg::OscpRaw`)
- `/oscp/quat` (`oscp_msgs::msg::OscpQuat`)
- `/oscp/euler` (`oscp_msgs::msg::OscpEuler`)
- `/oscp/rot` (`oscp_msgs::msg::OscpRot`)
- `/oscp/gnss` (`oscp_msgs::msg::OscpGnss`)

Standard ROS topics:

- `/oscp/imu/data` (`sensor_msgs::msg::Imu`)
- `/oscp/imu/mag` (`sensor_msgs::msg::MagneticField`)
- `/oscp/imu/temp` (`sensor_msgs::msg::Temperature`)

## QoS Guidance

For high-rate IMU telemetry, use `SensorDataQoS` in code for publishers/subscribers. Example (C++):

```cpp
auto qos = rclcpp::SensorDataQoS();
qos.keep_last(1000);

publisher = this->create_publisher<oscp_msgs::msg::OscpRaw>("/oscp/raw", qos);
```

Notes:

- The `ros2 topic hz` CLI in this ROS 2 distribution does not accept QoS arguments; it subscribes with default QoS. Use `ros2 topic echo` with a QoS profile for QoS-matching CLI checks, or run a small subscriber node.
- Example (CLI QoS-aware echo):

```bash
ros2 topic echo /oscp/raw --qos-profile sensor_data
```

## Debugging & CLI Examples

- Send magnetometer calibration action and follow feedback:

```bash
ros2 action send_goal /oscp/magnetometer_calibration oscp_imu_calibration/action/MagnetometerCalibration "{start: true}" --feedback
```

- Dump node parameters:

```bash
ros2 param dump /oscp_imu_node
```

- Call the `get_config` service and pretty-print the result:

```bash
ros2 service call /oscp/get_config oscp_imu_ros2/srv/GetIMUConfig "{}" | sed '\\'s/\\n/\n/g'```

