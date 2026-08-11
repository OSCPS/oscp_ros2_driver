from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="magcal_wrapper",
            executable="magnetometer_calibration",
            name="magnetometer_calibration_node",
            output="screen",
            emulate_tty=True,
        ),
    ])