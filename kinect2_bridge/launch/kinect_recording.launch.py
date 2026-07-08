#!/usr/bin/env python3
"""Kinect recording pipeline.

Starts colour video and rosbag recording immediately for cameras marked
record: true in the selected camera config.
"""

import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from kinect2_bridge.recording_config import (
    bag_topics_for_camera,
    color_topics,
    resolve_config_path,
    selected_cameras,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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

    output_root = recording_cfg["output_root"]
    participant_id = recording_cfg["uuid"]
    video_encoder = recording_cfg["video_encoder"]
    video_crf = recording_cfg["video_crf"]
    video_fps = recording_cfg["video_fps"]
    bag_storage = recording_cfg["bag_storage"]
    bag_compression = str(recording_cfg.get("bag_compression", "none")).lower()
    bag_cache_size_mb = int(recording_cfg.get("bag_cache_size_mb", 128))

    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(output_root, participant_id, f"session_{run_stamp}")
    os.makedirs(output_dir, exist_ok=True)

    actions = [
        Node(
            package="kinect2_bridge",
            executable="colour_video_recorder.py",
            name="colour_video_recorder",
            output="screen",
            parameters=[
                {
                    "topics": color_topics(cameras),
                    "output_dir": output_dir,
                    "encoder": video_encoder,
                    "crf": video_crf,
                    "fps": video_fps,
                }
            ],
        )
    ]

    for cam in cameras:
        ns = cam["namespace"]
        bag_dir = os.path.join(output_dir, f"{ns}_rosbag")
        bag_topics = bag_topics_for_camera(ns)

        bag_cmd = [
            "ros2",
            "bag",
            "record",
            "-o",
            bag_dir,
            "--storage",
            bag_storage,
            "--max-cache-size",
            str(bag_cache_size_mb * 1024 * 1024),
        ]
        if bag_compression in {"file", "message"}:
            bag_cmd += [
                "--compression-mode", bag_compression,
                "--compression-format", "zstd",
            ]
        bag_cmd += bag_topics

        actions.append(ExecuteProcess(cmd=bag_cmd, output="screen"))

    return actions


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
