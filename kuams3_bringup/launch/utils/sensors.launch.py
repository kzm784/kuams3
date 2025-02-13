from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():

    bringup_dir = get_package_share_directory("kuams3_bringup")

    # livox_ros_driver2 Node
    livox_ros_driver2_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_ros_driver2',
        output='screen',
        parameters=[
            {
                'xfer_format': 0,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'lvx_file_path': '',
                'user_config_path': os.path.join(
                    bringup_dir, 'config', 'MID360_config.json'
                ),
                'cmdline_input_bd_code': 'livox0000000001',
            }
        ]
    )

    #     # Pointcloud to LaserScan Node
    # pointcloud_to_laserscan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan',
    #     remappings=[('cloud_in', 'livox/lidar'),
    #                 ('scan', '/livox_scan')],
    #     parameters=[{
    #         'target_frame': 'livox_frame',
    #         'transform_tolerance': 0.01,
    #         'min_height': -0.2,
    #         'max_height': 0.2,
    #         'angle_min': -3.1415,
    #         'angle_max': 3.1415,
    #         'angle_increment': 0.0087,
    #         'scan_time': 0.3333,
    #         'range_min': 0.45,
    #         'range_max': 30.0,
    #         'use_inf': True,
    #         'inf_epsilon': 1.0
    #     }]
    # )

    
    return LaunchDescription([
        livox_ros_driver2_node,
        # pointcloud_to_laserscan_node
    ])