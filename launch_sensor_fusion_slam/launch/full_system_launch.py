from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='launch_sensor_fusion_slam').find('launch_sensor_fusion_slam')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'working_camera_image.rviz')  # Update with your actual RViz config file

    return LaunchDescription([
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),

        # SLAM Toolbox (online async SLAM)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('slam_toolbox').find('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                )
            ])
        ),

        # Custom sensor fusion localisation node
        Node(
            package='launch_sensor_fusion_slam',
            executable='localisation_test',
            name='localisation_node',
            output='screen',
        ),

        # LIDAR processing node
        Node(
            package='launch_sensor_fusion_slam',
            executable='lidar_test',
            name='lidar_node',
            output='screen',
        ),
    ])

