#!/usr/bin/env python3
"""Publish static TFs for cameras listed in multi_camera_config.yaml.

Reads the `cameras:` entries and publishes a static transform from the
configured `world_frame` (default `map`) to each camera `frame` using the
position/orientation fields.

Usage: static_tf_from_yaml.py <config_path>
"""

from __future__ import annotations

import os
import sys
from math import sin, cos

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from kinect2_bridge.recording_config import load_multi_camera_config, selected_cameras


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    # Convert roll/pitch/yaw (radians) to quaternion (x,y,z,w)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class StaticTfFromYaml(Node):
    """Publishes world->frame static TFs from a YAML config.

    Watches the config file for changes and re-publishes on save, so sensor
    poses can be adjusted by editing the YAML — no node restart needed.
    """

    def __init__(self, config_path: str, flag: str = "enabled"):
        super().__init__("static_tf_from_yaml")

        self._config_path = config_path
        self._flag = flag
        self._broadcaster = StaticTransformBroadcaster(self)
        self._last_mtime: float | None = None

        self._publish_from_config()
        self._watch_timer = self.create_timer(2.0, self._check_for_changes)

    def _check_for_changes(self):
        try:
            mtime = os.stat(self._config_path).st_mtime
        except OSError:
            return
        if self._last_mtime is not None and mtime != self._last_mtime:
            self.get_logger().info(f"{self._config_path} changed — republishing TFs")
            self._publish_from_config()

    def _publish_from_config(self):
        try:
            self._last_mtime = os.stat(self._config_path).st_mtime
            world_frame, cameras = load_multi_camera_config(self._config_path)
            _, selected = selected_cameras(self._config_path, self._flag)
        except Exception as exc:
            # Keep the previous transforms on a malformed edit (e.g. save
            # mid-typing); the next file change triggers another attempt.
            self.get_logger().error(f"Failed to read {self._config_path}: {exc}")
            return

        if not cameras:
            self.get_logger().warning(f"No cameras found in config: {self._config_path}")

        transforms: list[TransformStamped] = []
        for cam in selected:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = world_frame
            t.child_frame_id = cam.get("frame", f"{cam['namespace']}_link")
            t.transform.translation.x = float(cam.get("x", 0.0))
            t.transform.translation.y = float(cam.get("y", 0.0))
            t.transform.translation.z = float(cam.get("z", 0.0))
            roll = float(cam.get("roll", 0.0))
            pitch = float(cam.get("pitch", 0.0))
            yaw = float(cam.get("yaw", 0.0))
            qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            transforms.append(t)
            self.get_logger().info(f"Prepared static TF: {t.header.frame_id} -> {t.child_frame_id}")

        if transforms:
            # transient_local semantics: late joiners receive the latest set
            self._broadcaster.sendTransform(transforms)
            self.get_logger().info("Published static transforms from YAML")
        else:
            self.get_logger().warning("No enabled cameras to publish static TFs for.")


def main(argv=None):
    argv = argv or sys.argv[1:]
    if len(argv) < 1:
        print("Usage: static_tf_from_yaml.py <config_path>")
        return 2

    config_path = argv[0]

    rclpy.init()
    node = StaticTfFromYaml(config_path)
    try:
        # Keep node alive so that static transforms remain available to late-joining nodes
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
