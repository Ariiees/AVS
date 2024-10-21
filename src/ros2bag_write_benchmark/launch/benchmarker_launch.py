from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('ros2bag_write_benchmark'),
        'config',
        'benchmarker_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use for the benchmarker node'
        ),

        Node(
            package='ros2bag_write_benchmark',
            executable='benchmarker',
            name='benchmarker',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        )
    ])

