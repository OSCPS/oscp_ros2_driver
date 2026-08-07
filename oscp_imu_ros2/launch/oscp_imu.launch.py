from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file',
            default_value='default.yaml',
            description='IMU configuration YAML file'
        ),

        Node(
            package='oscp_imu_ros2',
            executable='oscp_imu_node',
            name='oscp_imu_node',

            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('oscp_imu_ros2'),
                    'config',
                    config_file
                ])
            ],

            output='screen'
        )
    ])
