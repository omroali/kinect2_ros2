#!/usr/bin/env python3
"""Launch the recording manager without starting a recording.

Use /start_recording and /stop_recording services to control capture remotely.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
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

    return [
        Node(
            package="kinect2_bridge",
            executable="recording_manager.py",
            name="recording_manager",
            output="screen",
            parameters=[{
                "output_root": OUTPUT_ROOT,
                "participant_id": UUID,
                "color_topics": color_topics(cameras),
                "bag_topics": [topic for cam in cameras for topic in bag_topics_for_camera(cam["namespace"])],
                "video_encoder": VIDEO_ENCODER,
                "video_crf": VIDEO_CRF,
                "video_fps": VIDEO_FPS,
                "bag_storage": BAG_STORAGE,
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
