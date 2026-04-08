#!/usr/bin/env python3

# Copyright (c) 2024
# Dual Kinect v2 Launch File for ROS 2
#
# This launch file starts two Kinect v2 sensors with static transforms
# between them for multi-camera point cloud fusion.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition


def generate_launch_description():
    # Camera 1 arguments
    camera1_namespace_arg = DeclareLaunchArgument(
        "camera1_namespace",
        default_value="kinect2_1",
        description="Namespace for first Kinect v2",
    )

    camera1_serial_arg = DeclareLaunchArgument(
        "camera1_serial",
        default_value="",
        description="Serial number for first Kinect v2 (empty for auto-detect)",
    )

    # Camera 2 arguments
    camera2_namespace_arg = DeclareLaunchArgument(
        "camera2_namespace",
        default_value="kinect2_2",
        description="Namespace for second Kinect v2",
    )

    camera2_serial_arg = DeclareLaunchArgument(
        "camera2_serial",
        default_value="",
        description="Serial number for second Kinect v2 (empty for auto-detect)",
    )

    # Transform arguments for camera 2 relative to camera 1
    camera2_x_arg = DeclareLaunchArgument(
        "camera2_x", default_value="1.0", description="Camera 2 X position in meters"
    )

    camera2_y_arg = DeclareLaunchArgument(
        "camera2_y", default_value="0.0", description="Camera 2 Y position in meters"
    )

    camera2_z_arg = DeclareLaunchArgument(
        "camera2_z", default_value="0.0", description="Camera 2 Z position in meters"
    )

    camera2_roll_arg = DeclareLaunchArgument(
        "camera2_roll", default_value="0.0", description="Camera 2 roll in radians"
    )

    camera2_pitch_arg = DeclareLaunchArgument(
        "camera2_pitch", default_value="0.0", description="Camera 2 pitch in radians"
    )

    camera2_yaw_arg = DeclareLaunchArgument(
        "camera2_yaw",
        default_value="-0.523599",
        description="Camera 2 yaw in radians (-30 degrees default)",
    )

    # Publish transforms argument
    publish_transforms_arg = DeclareLaunchArgument(
        "publish_transforms",
        default_value="true",
        description="Whether to publish static transforms between cameras",
    )

    # Get launch configurations
    camera1_namespace = LaunchConfiguration("camera1_namespace")
    camera1_serial = LaunchConfiguration("camera1_serial")
    camera2_namespace = LaunchConfiguration("camera2_namespace")
    camera2_serial = LaunchConfiguration("camera2_serial")
    camera2_x = LaunchConfiguration("camera2_x")
    camera2_y = LaunchConfiguration("camera2_y")
    camera2_z = LaunchConfiguration("camera2_z")
    camera2_roll = LaunchConfiguration("camera2_roll")
    camera2_pitch = LaunchConfiguration("camera2_pitch")
    camera2_yaw = LaunchConfiguration("camera2_yaw")
    publish_transforms = LaunchConfiguration("publish_transforms")

    # Camera 1 - Kinect v2 Bridge
    camera1_group = GroupAction(
        [
            PushRosNamespace(camera1_namespace),
            Node(
                package="kinect2_bridge",
                executable="kinect2_bridge_node",
                name="kinect2_bridge",
                output="screen",
                parameters=[
                    {
                        "base_name": camera1_namespace,
                        "sensor": camera1_serial,
                        "publish_tf": True,
                        "base_name_tf": [camera1_namespace, "_link"],
                        "fps_limit": 30.0,
                        "use_png": False,
                        "depth_method": "default",
                        "reg_method": "default",
                        "max_depth": 12.0,
                        "min_depth": 0.1,
                        "queue_size": 5,
                        "bilateral_filter": True,
                        "edge_aware_filter": True,
                        "worker_threads": 4,
                    }
                ],
            ),
            # Point cloud for SD resolution (512x424)
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyzrgb_node",
                name="points_xyzrgb_sd",
                remappings=[
                    ("rgb/camera_info", "sd/camera_info"),
                    ("rgb/image_rect_color", "sd/image_color_rect"),
                    ("depth_registered/image_rect", "sd/image_depth_rect"),
                    ("points", "sd/points"),
                ],
            ),
            # Point cloud for QHD resolution (960x540)
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyzrgb_node",
                name="points_xyzrgb_qhd",
                remappings=[
                    ("rgb/camera_info", "qhd/camera_info"),
                    ("rgb/image_rect_color", "qhd/image_color_rect"),
                    ("depth_registered/image_rect", "qhd/image_depth_rect"),
                    ("points", "qhd/points"),
                ],
            ),
        ]
    )

    # Camera 2 - Kinect v2 Bridge
    camera2_group = GroupAction(
        [
            PushRosNamespace(camera2_namespace),
            Node(
                package="kinect2_bridge",
                executable="kinect2_bridge_node",
                name="kinect2_bridge",
                output="screen",
                parameters=[
                    {
                        "base_name": camera2_namespace,
                        "sensor": camera2_serial,
                        "publish_tf": True,
                        "base_name_tf": [camera2_namespace, "_link"],
                        "fps_limit": 30.0,
                        "use_png": False,
                        "depth_method": "default",
                        "reg_method": "default",
                        "max_depth": 12.0,
                        "min_depth": 0.1,
                        "queue_size": 5,
                        "bilateral_filter": True,
                        "edge_aware_filter": True,
                        "worker_threads": 4,
                    }
                ],
            ),
            # Point cloud for SD resolution
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyzrgb_node",
                name="points_xyzrgb_sd",
                remappings=[
                    ("rgb/camera_info", "sd/camera_info"),
                    ("rgb/image_rect_color", "sd/image_color_rect"),
                    ("depth_registered/image_rect", "sd/image_depth_rect"),
                    ("points", "sd/points"),
                ],
            ),
            # Point cloud for QHD resolution
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyzrgb_node",
                name="points_xyzrgb_qhd",
                remappings=[
                    ("rgb/camera_info", "qhd/camera_info"),
                    ("rgb/image_rect_color", "qhd/image_color_rect"),
                    ("depth_registered/image_rect", "qhd/image_depth_rect"),
                    ("points", "qhd/points"),
                ],
            ),
        ]
    )

    # World frame publisher (fixed reference) - uses map as parent frame
    world_frame_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_frame_publisher",
        arguments=[
            "--frame-id",
            "map",
            "--child-frame-id",
            [camera1_namespace, "_link"],
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
        ],
        condition=IfCondition(publish_transforms),
    )

    # Camera 2 transform relative to map frame
    camera2_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera2_transform_publisher",
        arguments=[
            "--frame-id",
            "map",
            "--child-frame-id",
            [camera2_namespace, "_link"],
            "--x",
            camera2_x,
            "--y",
            camera2_y,
            "--z",
            camera2_z,
            "--roll",
            camera2_roll,
            "--pitch",
            camera2_pitch,
            "--yaw",
            camera2_yaw,
        ],
        condition=IfCondition(publish_transforms),
    )

    return LaunchDescription(
        [
            # Declare arguments
            camera1_namespace_arg,
            camera1_serial_arg,
            camera2_namespace_arg,
            camera2_serial_arg,
            camera2_x_arg,
            camera2_y_arg,
            camera2_z_arg,
            camera2_roll_arg,
            camera2_pitch_arg,
            camera2_yaw_arg,
            publish_transforms_arg,
            # Launch camera nodes
            camera1_group,
            camera2_group,
            # Publish transforms
            world_frame_publisher,
            camera2_transform_publisher,
        ]
    )
