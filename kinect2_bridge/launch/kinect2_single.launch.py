#!/usr/bin/env python3

# Copyright (c) 2024
# Single Kinect v2 Launch File for ROS 2
#
# This launch file starts a single Kinect v2 sensor.
# It uses quoted string format for the sensor parameter to force string type,
# avoiding YAML type conversion issues with serial numbers that have leading
# zeros (e.g., 001934470647).
#
# Usage:
#   ros2 launch kinect2_bridge kinect2_single.launch.py serial:=007425354147 namespace:=kinect2_1
#   ros2 launch kinect2_bridge kinect2_single.launch.py serial:=001934470647 namespace:=kinect2_2
#
# For dynamic camera positioning (adjustable at runtime):
#   ros2 launch kinect2_bridge kinect2_single.launch.py serial:=007425354147 namespace:=kinect2_1 use_dynamic_tf:=true
#   Then adjust positions with:
#     ros2 param set /dynamic_camera_tf camera1_x 0.5
#     ros2 param set /dynamic_camera_tf camera1_yaw 0.3
#   Or use: ros2 run rqt_reconfigure rqt_reconfigure

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def launch_kinect_setup(context, *args, **kwargs):
    """
    OpaqueFunction callback that launches Kinect nodes.
    Uses quoted string format in ros_arguments to force string type,
    avoiding YAML type conversion issues with leading zeros.
    """
    # Get the evaluated string values from LaunchConfiguration
    serial = LaunchConfiguration('serial').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    use_dynamic_tf = LaunchConfiguration('use_dynamic_tf').perform(context).lower() == 'true'

    # Camera position parameters (for static TF)
    cam_x = float(LaunchConfiguration('x').perform(context))
    cam_y = float(LaunchConfiguration('y').perform(context))
    cam_z = float(LaunchConfiguration('z').perform(context))
    cam_roll = float(LaunchConfiguration('roll').perform(context))
    cam_pitch = float(LaunchConfiguration('pitch').perform(context))
    cam_yaw = float(LaunchConfiguration('yaw').perform(context))

    # Frame names - used for TF
    base_frame = f"{namespace}_link"

    # Determine camera number from namespace (for dynamic TF)
    camera_num = '1' if '1' in namespace else '2'

    # Build ros_arguments to pass sensor parameter via command line
    # Use single quotes around the value to force ROS 2 to interpret as string
    # This avoids YAML/numeric conversion issues with leading zeros like 001934470647
    ros_args = [
        '-p', f"sensor:='{serial}'",
    ]

    # Kinect2 Bridge Node
    # NOTE: We set base_name to namespace so topics are at /{namespace}/hd, /{namespace}/sd, etc.
    # We do NOT use ROS namespace to avoid double-namespacing (/{ns}/{base_name}/...)
    kinect2_bridge_node = Node(
        package='kinect2_bridge',
        executable='kinect2_bridge_node',
        name=f'{namespace}_bridge',
        # No namespace here - base_name handles topic prefixing
        output='screen',
        ros_arguments=ros_args,
        parameters=[{
            'base_name': namespace,  # Topics will be at /{namespace}/hd, /sd, /qhd
            'publish_tf': True,
            'base_name_tf': namespace,  # TF frames: kinect2_bridge appends _link, _rgb_optical_frame, etc.
            'fps_limit': 30.0,
            'use_png': False,
            'depth_method': 'default',
            'reg_method': 'default',
            'max_depth': 12.0,
            'min_depth': 0.1,
            'queue_size': 5,
            'bilateral_filter': True,
            'edge_aware_filter': True,
            'worker_threads': 4,
        }],
    )

    # Point cloud for SD resolution (512x424)
    # Remappings connect depth_image_proc inputs to kinect2_bridge outputs
    points_xyzrgb_sd_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='points_xyzrgb_sd',
        namespace=namespace,
        output='screen',
        parameters=[{
            'queue_size': 10,
        }],
        remappings=[
            # Input topics - from kinect2_bridge (absolute paths)
            ('rgb/camera_info', f'/{namespace}/sd/camera_info'),
            ('rgb/image_rect_color', f'/{namespace}/sd/image_color_rect'),
            ('depth_registered/image_rect', f'/{namespace}/sd/image_depth_rect'),
            # Output topic - relative to namespace
            ('points', 'sd/points'),
        ],
    )

    # Point cloud for QHD resolution (960x540)
    points_xyzrgb_qhd_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='points_xyzrgb_qhd',
        namespace=namespace,
        output='screen',
        parameters=[{
            'queue_size': 10,
        }],
        remappings=[
            # Input topics - from kinect2_bridge (absolute paths)
            ('rgb/camera_info', f'/{namespace}/qhd/camera_info'),
            ('rgb/image_rect_color', f'/{namespace}/qhd/image_color_rect'),
            ('depth_registered/image_rect', f'/{namespace}/qhd/image_depth_rect'),
            # Output topic - relative to namespace
            ('points', 'qhd/points'),
        ],
    )

    # Build the node list
    nodes = [
        kinect2_bridge_node,
        points_xyzrgb_sd_node,
        points_xyzrgb_qhd_node,
    ]

    if use_dynamic_tf:
        # Dynamic TF broadcaster - positions can be adjusted at runtime
        # via ros2 param set or rqt_reconfigure
        dynamic_tf_node = Node(
            package='kinect2_bridge',
            executable='dynamic_camera_tf.py',
            name='dynamic_camera_tf',
            output='screen',
            parameters=[{
                f'camera{camera_num}_frame': base_frame,
                f'camera{camera_num}_x': cam_x,
                f'camera{camera_num}_y': cam_y,
                f'camera{camera_num}_z': cam_z,
                f'camera{camera_num}_roll': cam_roll,
                f'camera{camera_num}_pitch': cam_pitch,
                f'camera{camera_num}_yaw': cam_yaw,
                f'camera{camera_num}_enabled': True,
                # Disable the other camera in single mode
                f'camera{"2" if camera_num == "1" else "1"}_enabled': False,
                'publish_rate': 50.0,
            }],
        )
        nodes.append(dynamic_tf_node)
    else:
        # Static transform publisher: world -> base_frame
        # The kinect2_bridge publishes: base_frame -> optical frames
        static_tf_world = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{namespace}_world_tf',
            arguments=[
                '--frame-id', 'world',
                '--child-frame-id', base_frame,
                '--x', str(cam_x),
                '--y', str(cam_y),
                '--z', str(cam_z),
                '--roll', str(cam_roll),
                '--pitch', str(cam_pitch),
                '--yaw', str(cam_yaw),
            ],
        )
        nodes.append(static_tf_world)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'serial',
            default_value='',
            description='Serial number of the Kinect v2 sensor (e.g., 007425354147 or 001934470647). '
                        'Leave empty for auto-detect (single Kinect only).'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='kinect2',
            description='Namespace/prefix for topics and TF frames (e.g., kinect2_1). '
                        'Topics will be at /{namespace}/hd, /{namespace}/sd, etc.'
        ),

        # Camera position arguments
        DeclareLaunchArgument(
            'x', default_value='0.0',
            description='Camera X position in meters'
        ),
        DeclareLaunchArgument(
            'y', default_value='0.0',
            description='Camera Y position in meters'
        ),
        DeclareLaunchArgument(
            'z', default_value='0.0',
            description='Camera Z position in meters'
        ),
        DeclareLaunchArgument(
            'roll', default_value='0.0',
            description='Camera roll angle in radians'
        ),
        DeclareLaunchArgument(
            'pitch', default_value='0.0',
            description='Camera pitch angle in radians'
        ),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Camera yaw angle in radians'
        ),

        # Dynamic TF option
        DeclareLaunchArgument(
            'use_dynamic_tf',
            default_value='false',
            description='Use dynamic TF broadcaster (allows runtime position adjustment via ros2 param or rqt_reconfigure)'
        ),

        # Use OpaqueFunction to build nodes with proper string handling
        OpaqueFunction(function=launch_kinect_setup),
    ])
