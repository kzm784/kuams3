import os
from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_package_path = get_package_share_path('kuams3_bringup')
    pub_odom_config_path = bringup_package_path / 'config/config_pub_odom.yaml'
    description_package_path = get_package_share_path('kuams3_description')
    default_model_path = description_package_path / 'urdf/kuams3.xacro'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    pub_odom_node = Node(
        package='kuams3_bringup',
        executable='pub_odom',
        name='pub_odom',
        parameters=[pub_odom_config_path]
    )

    kuams_bringup_dir = get_package_share_path('kuams3_bringup')
    sensors_launch_file = os.path.join(kuams_bringup_dir, 'launch', 'utils', 'sensors.launch.py')

    include_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_file)
    )

    ld = LaunchDescription()
    ld.add_action(model_arg)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(pub_odom_node)
    ld.add_action(include_sensors_launch)

    return ld
