#!/usr/bin/env python3
"""Kinect recording pipeline.

Records colour topics to H.265 MP4 via colour_video_recorder.py and records
depth + camera_info + TF into one rosbag per active camera.

Edit the constants below to choose:
- which cameras are active
- the output directory
- the video codec/quality/FPS
- the bag topics to capture
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node

from kinect2_bridge.recording_config import bag_topics_for_camera, color_topics, selected_cameras


OUTPUT_ROOT = os.path.expanduser("~/data")
UUID = "0000"
VIDEO_ENCODER = "libx265"
VIDEO_CRF = 18
VIDEO_FPS = 30.0
BAG_STORAGE = "mcap"
CONFIG_PATH = os.path.join(
    get_package_share_directory("kinect2_bridge"),
    "config",
    "multi_camera_config.yaml",
)


def launch_setup(context, *args, **kwargs):
    del context, args, kwargs
    _, cameras = selected_cameras(CONFIG_PATH, "record")
    if not cameras:
        raise RuntimeError("No cameras are marked record: true in multi_camera_config.yaml.")

    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(OUTPUT_ROOT, UUID, f"session_{run_stamp}")
    os.makedirs(output_dir, exist_ok=True)

    actions = []

    actions.append(
        Node(
            package="kinect2_bridge",
            executable="colour_video_recorder.py",
            name="colour_video_recorder",
            output="screen",
            parameters=[{
                "topics": color_topics(cameras),
                "output_dir": output_dir,
                "encoder": VIDEO_ENCODER,
                "crf": VIDEO_CRF,
                "fps": VIDEO_FPS,
            }],
        )
    )

    for cam in cameras:
        ns = cam["namespace"]
        bag_dir = os.path.join(output_dir, f"{ns}_rosbag")
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    "-o", bag_dir,
                    "--storage", BAG_STORAGE,
                    *bag_topics_for_camera(ns),
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
