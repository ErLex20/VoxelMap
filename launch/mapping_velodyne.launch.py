"""Launch file for Velodyne VLP-16 LiDAR."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('voxel_map'), 'config', 'velodyne.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='voxel_map', executable='voxel_mapping_odom',
             name='voxel_mapping_odom', output='screen', parameters=[config]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(get_package_share_directory('voxel_map'),
                                           'rviz_cfg', 'voxel_mapping.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
