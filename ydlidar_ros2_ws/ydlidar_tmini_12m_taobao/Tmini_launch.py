import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'Tmini.yaml'),
                                           description='Path to the ROS2 parameters file to use.')

    return LaunchDescription([
        params_declare,
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            parameters=[LaunchConfiguration('params_file')]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'laser_frame'],
        )
    ])
