#!/usr/bin/env python3
"""Launch the recording manager without starting a recording.

Use /start_recording and /stop_recording services to control capture remotely.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from kinect2_bridge.recording_config import (
    bag_topics_for_camera,
    color_topics,
    resolve_config_path,
    selected_cameras,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    config_file = LaunchConfiguration("config_file").perform(context)
    recording_config_file = LaunchConfiguration("recording_config").perform(context)

    _, cameras = selected_cameras(config_file, "record")
    if not cameras:
        raise RuntimeError(f"No cameras are marked record: true in {config_file}.")

    with open(recording_config_file, "r") as f:
        recording_cfg = yaml.safe_load(f)["recording_settings"]

    return [
        Node(
            package="kinect2_bridge",
            executable="recording_manager.py",
            name="recording_manager",
            output="screen",
            parameters=[
                {
                    "output_root": recording_cfg["output_root"],
                    "participant_id": recording_cfg["uuid"],
                    "color_topics": color_topics(cameras),
                    "bag_topics": [
                        topic
                        for cam in cameras
                        for topic in bag_topics_for_camera(cam["namespace"])
                    ],
                    "video_encoder": recording_cfg["video_encoder"],
                    "video_crf": recording_cfg["video_crf"],
                    "video_fps": recording_cfg["video_fps"],
                    "bag_storage": recording_cfg["bag_storage"],
                    "bag_compression": recording_cfg.get("bag_compression", "none"),
                    "bag_cache_size_mb": int(
                        recording_cfg.get("bag_cache_size_mb", 128)
                    ),
                }
            ],
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("kinect2_bridge")
    default_config_file = resolve_config_path(
        "kinect_cameras.yaml",
        os.path.join(pkg_share, "config", "multi_camera_config.yaml"),
    )
    default_recording_config = resolve_config_path(
        "recording.yaml",
        os.path.join(pkg_share, "config", "recording_config.yaml"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to multi-camera selection config.",
            ),
            DeclareLaunchArgument(
                "recording_config",
                default_value=default_recording_config,
                description="Path to generic recording settings YAML.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
