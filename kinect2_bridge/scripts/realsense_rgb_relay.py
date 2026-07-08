#!/usr/bin/env python3
"""Lightweight static Realsense RGB relay.

Relays only RGB-related topics for configured cameras into:
  /realsense_cameras/<namespace>/rgb/image_raw
  /realsense_cameras/<namespace>/rgb/image_raw/compressed
  /realsense_cameras/<namespace>/rgb/camera_info
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

from kinect2_bridge.recording_config import selected_cameras


class RealsenseRgbRelay(Node):
    def __init__(self, config_path: str, camera_flag: str = 'enabled'):
        super().__init__('realsense_rgb_relay')

        _, cameras = selected_cameras(config_path, camera_flag)
        if not cameras:
            raise RuntimeError(f"No cameras selected with flag '{camera_flag}' from: {config_path}")

        self._pubs = []
        self._subs = []

        for cam in cameras:
            ns = cam['namespace']
            serial = cam['serial']
            frame = cam['frame']

            src_root = f"/realsense/D555_{serial}"
            dst_root = f"/realsense_cameras/{ns}/rgb"

            src_raw = f"{src_root}_Color"
            src_raw_info = f"{src_root}_Color/camera_info"
            src_comp = f"{src_root}_CompressedColor"
            src_comp_info = f"{src_root}_CompressedColor/camera_info"

            dst_comp = f"{dst_root}/image_raw/compressed"
            dst_comp_info = f"{dst_root}/camera_info"

            comp_pub = self.create_publisher(CompressedImage, dst_comp, qos_profile_sensor_data)
            info_pub = self.create_publisher(CameraInfo, dst_comp_info, qos_profile_sensor_data)

            self._pubs.extend([comp_pub, info_pub])

            def comp_cb(msg, pub=comp_pub, frame_id=frame):
                if hasattr(msg, 'header'):
                    msg.header.frame_id = frame_id
                pub.publish(msg)

            def info_cb(msg, pub=info_pub, frame_id=frame):
                if hasattr(msg, 'header'):
                    msg.header.frame_id = frame_id
                pub.publish(msg)

            # Subscribe only to the compressed image stream and its camera_info to avoid
            # expensive compressed<->raw conversions in Python. Consumers should use
            # image_transport with the 'compressed' hint or subscribe to the
            # '/image_raw/compressed' topic directly.
            self._subs.append(self.create_subscription(CompressedImage, src_comp, comp_cb, qos_profile_sensor_data))
            # Some drivers publish compressed camera info on a separate topic.
            self._subs.append(self.create_subscription(CameraInfo, src_comp_info, info_cb, qos_profile_sensor_data))

            self.get_logger().info(
                f"RGB relay {src_comp} -> {dst_comp}, frame_id={frame}"
            )


def main(argv=None):
    argv = argv or sys.argv[1:]
    if len(argv) < 1:
        print('Usage: realsense_rgb_relay.py <config_path> [camera_flag]')
        return 2

    config_path = argv[0]
    camera_flag = argv[1] if len(argv) > 1 else 'enabled'

    rclpy.init()
    node = RealsenseRgbRelay(config_path, camera_flag)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
