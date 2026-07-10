from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # CONFIG FILE ARG
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([

        # CONFIG SELECTOR
        DeclareLaunchArgument('config_file', default_value='test.yaml', description='IMU configuration YAML file'),

        # RUNTIME / ROS SETTINGS
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),

        # FRAME OUTPUTS
        DeclareLaunchArgument('pub_accel_in_g', default_value='false'),
        DeclareLaunchArgument('oscp_raw_enabled', default_value='true'),
        DeclareLaunchArgument('oscp_quat_enabled', default_value='true'),
        DeclareLaunchArgument('oscp_euler_enabled', default_value='true'),
        DeclareLaunchArgument('oscp_rotation_matrix_enabled', default_value='true'),
        DeclareLaunchArgument('standard_ros_enabled', default_value='true'),

        # Node Execution 
        Node(package='oscp_imu_ros2', executable='oscp_imu_node', name='oscp_imu_node',

            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('oscp_imu_ros2'),
                    'config',
                    config_file
                ]),

                {
                    'device': LaunchConfiguration('device'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'pub_accel_in_g': LaunchConfiguration('pub_accel_in_g'),

                    'oscp_raw_enabled': LaunchConfiguration('oscp_raw_enabled'),
                    'oscp_quat_enabled': LaunchConfiguration('oscp_quat_enabled'),
                    'oscp_euler_enabled': LaunchConfiguration('oscp_euler_enabled'),
                    'oscp_rotation_matrix_enabled': LaunchConfiguration('oscp_rotation_matrix_enabled'),
                    'standard_ros_enabled': LaunchConfiguration('standard_ros_enabled'),
                }
            ],

            output='screen'
        )
    ])