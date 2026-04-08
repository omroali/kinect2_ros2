#!/usr/bin/env python3

# Copyright (c) 2024
# Dual Kinect v2 Launch File with Dynamic TF Positioning
#
# This launch file starts two Kinect v2 sensors with dynamic TF transforms
# that can be adjusted at runtime using ros2 param or rqt_reconfigure.
#
# Usage:
#   ros2 launch kinect2_bridge kinect2_dual_dynamic.launch.py
#
#   # With custom serials:
#   ros2 launch kinect2_bridge kinect2_dual_dynamic.launch.py \
#       serial1:=007425354147 serial2:=001934470647
#
#   # With initial positions:
#   ros2 launch kinect2_bridge kinect2_dual_dynamic.launch.py \
#       camera1_x:=0.0 camera1_y:=0.0 camera2_x:=1.0 camera2_y:=0.5 camera2_yaw:=-0.5
#
# To adjust positions at runtime:
#   ros2 param set /dynamic_camera_tf camera1_x 0.5
#   ros2 param set /dynamic_camera_tf camera1_yaw 0.3
#   ros2 param set /dynamic_camera_tf camera2_x 1.5
#   ros2 param set /dynamic_camera_tf camera2_y -0.5
#
# Or use GUI:
#   ros2 run rqt_reconfigure rqt_reconfigure

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
    OpaqueFunction callback that launches both Kinect nodes with dynamic TF.
    """
    # Get parameters
    serial1 = LaunchConfiguration('serial1').perform(context)
    serial2 = LaunchConfiguration('serial2').perform(context)
    namespace1 = LaunchConfiguration('namespace1').perform(context)
    namespace2 = LaunchConfiguration('namespace2').perform(context)

    # Camera 1 position
    cam1_x = float(LaunchConfiguration('camera1_x').perform(context))
    cam1_y = float(LaunchConfiguration('camera1_y').perform(context))
    cam1_z = float(LaunchConfiguration('camera1_z').perform(context))
    cam1_roll = float(LaunchConfiguration('camera1_roll').perform(context))
    cam1_pitch = float(LaunchConfiguration('camera1_pitch').perform(context))
    cam1_yaw = float(LaunchConfiguration('camera1_yaw').perform(context))

    # Camera 2 position
    cam2_x = float(LaunchConfiguration('camera2_x').perform(context))
    cam2_y = float(LaunchConfiguration('camera2_y').perform(context))
    cam2_z = float(LaunchConfiguration('camera2_z').perform(context))
    cam2_roll = float(LaunchConfiguration('camera2_roll').perform(context))
    cam2_pitch = float(LaunchConfiguration('camera2_pitch').perform(context))
    cam2_yaw = float(LaunchConfiguration('camera2_yaw').perform(context))

    # Frame names
    base_frame1 = f"{namespace1}_link"
    base_frame2 = f"{namespace2}_link"

    # Build ros_arguments for serial parameters (to avoid YAML type conversion issues)
    ros_args1 = ['-p', f"sensor:='{serial1}'"]
    ros_args2 = ['-p', f"sensor:='{serial2}'"]

    nodes = []

    # ========== Kinect 1 ==========
    kinect1_bridge = Node(
        package='kinect2_bridge',
        executable='kinect2_bridge_node',
        name=f'{namespace1}_bridge',
        output='screen',
        ros_arguments=ros_args1,
        parameters=[{
            'base_name': namespace1,
            'publish_tf': True,
            'base_name_tf': namespace1,
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
    nodes.append(kinect1_bridge)

    # Kinect 1 point clouds
    kinect1_points_sd = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='points_xyzrgb_sd',
        namespace=namespace1,
        output='screen',
        parameters=[{'queue_size': 10}],
        remappings=[
            ('rgb/camera_info', f'/{namespace1}/sd/camera_info'),
            ('rgb/image_rect_color', f'/{namespace1}/sd/image_color_rect'),
            ('depth_registered/image_rect', f'/{namespace1}/sd/image_depth_rect'),
            ('points', 'sd/points'),
        ],
    )
    nodes.append(kinect1_points_sd)

    kinect1_points_qhd = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='points_xyzrgb_qhd',
        namespace=namespace1,
        output='screen',
        parameters=[{'queue_size': 10}],
        remappings=[
            ('rgb/camera_info', f'/{namespace1}/qhd/camera_info'),
            ('rgb/image_rect_color', f'/{namespace1}/qhd/image_color_rect'),
            ('depth_registered/image_rect', f'/{namespace1}/qhd/image_depth_rect'),
            ('points', 'qhd/points'),
        ],
    )
    nodes.append(kinect1_points_qhd)

    # ========== Kinect 2 (delayed start for USB stability) ==========
    kinect2_bridge = Node(
        package='kinect2_bridge',
        executable='kinect2_bridge_node',
        name=f'{namespace2}_bridge',
        output='screen',
        ros_arguments=ros_args2,
        parameters=[{
            'base_name': namespace2,
            'publish_tf': True,
            'base_name_tf': namespace2,
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

    # Kinect 2 point clouds
    kinect2_points_sd = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='points_xyzrgb_sd',
        namespace=namespace2,
        output='screen',
        parameters=[{'queue_size': 10}],
        remappings=[
            ('rgb/camera_info', f'/{namespace2}/sd/camera_info'),
            ('rgb/image_rect_color', f'/{namespace2}/sd/image_color_rect'),
            ('depth_registered/image_rect', f'/{namespace2}/sd/image_depth_rect'),
            ('points', 'sd/points'),
        ],
    )

    kinect2_points_qhd = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='points_xyzrgb_qhd',
        namespace=namespace2,
        output='screen',
        parameters=[{'queue_size': 10}],
        remappings=[
            ('rgb/camera_info', f'/{namespace2}/qhd/camera_info'),
            ('rgb/image_rect_color', f'/{namespace2}/qhd/image_color_rect'),
            ('depth_registered/image_rect', f'/{namespace2}/qhd/image_depth_rect'),
            ('points', 'qhd/points'),
        ],
    )

    # Delay Kinect 2 startup by 5 seconds for USB stability
    kinect2_delayed = TimerAction(
        period=5.0,
        actions=[kinect2_bridge, kinect2_points_sd, kinect2_points_qhd]
    )
    nodes.append(kinect2_delayed)

    # ========== Dynamic TF Broadcaster ==========
    # This single node publishes TF for both cameras
    # Positions can be adjusted at runtime via ros2 param or rqt_reconfigure
    dynamic_tf = Node(
        package='kinect2_bridge',
        executable='dynamic_camera_tf.py',
        name='dynamic_camera_tf',
        output='screen',
        parameters=[{
            # Camera 1
            'camera1_frame': base_frame1,
            'camera1_x': cam1_x,
            'camera1_y': cam1_y,
            'camera1_z': cam1_z,
            'camera1_roll': cam1_roll,
            'camera1_pitch': cam1_pitch,
            'camera1_yaw': cam1_yaw,
            'camera1_enabled': True,
            # Camera 2
            'camera2_frame': base_frame2,
            'camera2_x': cam2_x,
            'camera2_y': cam2_y,
            'camera2_z': cam2_z,
            'camera2_roll': cam2_roll,
            'camera2_pitch': cam2_pitch,
            'camera2_yaw': cam2_yaw,
            'camera2_enabled': True,
            # Settings
            'world_frame': 'map',
            'publish_rate': 50.0,
        }],
    )
    nodes.append(dynamic_tf)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ========== Serial Numbers ==========
        DeclareLaunchArgument(
            'serial1',
            default_value='007425354147',
            description='Serial number for Kinect 1'
        ),
        DeclareLaunchArgument(
            'serial2',
            default_value='001934470647',
            description='Serial number for Kinect 2'
        ),

        # ========== Namespaces ==========
        DeclareLaunchArgument(
            'namespace1',
            default_value='kinect2_1',
            description='Namespace for Kinect 1'
        ),
        DeclareLaunchArgument(
            'namespace2',
            default_value='kinect2_2',
            description='Namespace for Kinect 2'
        ),

        # ========== Camera 1 Position ==========
        DeclareLaunchArgument(
            'camera1_x', default_value='0.0',
            description='Camera 1 X position in meters'
        ),
        DeclareLaunchArgument(
            'camera1_y', default_value='0.0',
            description='Camera 1 Y position in meters'
        ),
        DeclareLaunchArgument(
            'camera1_z', default_value='0.0',
            description='Camera 1 Z position in meters'
        ),
        DeclareLaunchArgument(
            'camera1_roll', default_value='0.0',
            description='Camera 1 roll angle in radians'
        ),
        DeclareLaunchArgument(
            'camera1_pitch', default_value='0.0',
            description='Camera 1 pitch angle in radians'
        ),
        DeclareLaunchArgument(
            'camera1_yaw', default_value='0.0',
            description='Camera 1 yaw angle in radians'
        ),

        # ========== Camera 2 Position ==========
        DeclareLaunchArgument(
            'camera2_x', default_value='1.0',
            description='Camera 2 X position in meters'
        ),
        DeclareLaunchArgument(
            'camera2_y', default_value='0.0',
            description='Camera 2 Y position in meters'
        ),
        DeclareLaunchArgument(
            'camera2_z', default_value='0.0',
            description='Camera 2 Z position in meters'
        ),
        DeclareLaunchArgument(
            'camera2_roll', default_value='0.0',
            description='Camera 2 roll angle in radians'
        ),
        DeclareLaunchArgument(
            'camera2_pitch', default_value='0.0',
            description='Camera 2 pitch angle in radians'
        ),
        DeclareLaunchArgument(
            'camera2_yaw', default_value='-0.5236',
            description='Camera 2 yaw angle in radians (-30 degrees default)'
        ),

        # Launch setup
        OpaqueFunction(function=launch_setup),
    ])
