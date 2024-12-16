import os
from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200']
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

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

    sensors_launch_file = os.path.join(bringup_package_path, 'launch', 'utils', 'sensors.launch.py')
    include_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_file)
    )

    dependent_nodes = TimerAction(
        period=5.0,
        actions=[
            joint_state_publisher_node,
            robot_state_publisher_node,
            pub_odom_node,
            include_sensors_launch,
        ]
    )

    ld = LaunchDescription()
    ld.add_action(micro_ros_agent_node)
    ld.add_action(model_arg)
    ld.add_action(dependent_nodes)

    return ld
