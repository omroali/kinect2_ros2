"""Launch static TF publisher from multi-camera YAML."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_path', default_value='config/multi_camera_config.yaml', description='Path to camera config'
    )

    config_path = LaunchConfiguration('config_path')

    node = Node(
        package='kinect2_bridge',
        executable='static_tf_from_yaml.py',
        name='static_tf_from_yaml',
        output='screen',
        arguments=[config_path],
    )

    return LaunchDescription([config_arg, node])
