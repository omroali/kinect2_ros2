#!/usr/bin/env python3
"""Relay Realsense topics under /realsense/D555_<serial>/... with configured frames.

This node watches ROS topics, sets up subscribers for topics under each
camera namespace from the YAML config, and republishes them under
/realsense/D555_<serial>/..., adjusting message header.frame_id to the
`frame` value from the config.
"""

from __future__ import annotations

import sys
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from kinect2_bridge.recording_config import selected_cameras


QOS_TF_STATIC = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

QOS_RELAY_PUB = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


ALLOWED_SUFFIXES = {
    '/tf_static',
    '/Color',
    '/Color/camera_info',
    '/Color/metadata',
}

ENABLE_DIAGNOSTICS = False


def _msg_class_from_type(type_str: str):
    # type_str like 'sensor_msgs/msg/Image'
    try:
        pkg, _, cls = type_str.partition('/msg/')
        if not cls:
            pkg, _, cls = type_str.partition('/msg/')
        module = pkg + '.msg'
        comp = cls.replace('/', '.')
        parts = comp.split('.')
        # import module and get class
        mod = __import__(module, fromlist=parts)
        return getattr(mod, parts[-1])
    except Exception:
        return None


class RealsenseRelay(Node):
    def __init__(self, config_path: str, camera_flag: str = 'enabled'):
        super().__init__('realsense_relay')
        self.config_path = config_path
        self.camera_flag = camera_flag

        _, cameras = selected_cameras(self.config_path, self.camera_flag)
        # map namespace -> entry
        self.cameras: Dict[str, Dict[str, Any]] = {cam['namespace']: cam for cam in cameras}

        self.get_logger().info(f"Loaded {len(self.cameras)} cameras from config")

        # track already created subscriptions by topic
        self._subs = {}
        self._pubs = {}
        self._seen_matches = 0

        # periodic timer to scan topics
        self.create_timer(1.0, self._scan_topics)
        # timer to report message counts for diagnostics
        self._counters: Dict[str, Dict[str, int]] = {}
        if ENABLE_DIAGNOSTICS:
            self.create_timer(5.0, self._report_counts)

    @staticmethod
    def _source_roots(cam: Dict[str, Any]) -> list[str]:
        # Support both naming schemes:
        # 1) /<namespace>/...
        # 2) /realsense/D555_<serial>...   (e.g. /realsense/D555_123_Color)
        return [
            f"/{cam['namespace']}",
            f"/realsense/D555_{cam['serial']}",
        ]

    @staticmethod
    def _normalize_suffix(suffix: str) -> str:
        if not suffix:
            return ''
        if suffix.startswith('_'):
            # D555_<serial>_Color -> /Color
            return '/' + suffix[1:]
        if not suffix.startswith('/'):
            return '/' + suffix
        return suffix

    def _match_camera_topic(self, topic: str):
        for cam in self.cameras.values():
            for root in self._source_roots(cam):
                if topic == root or topic.startswith(root):
                    suffix = topic[len(root):]
                    return cam, root, suffix
        return None, None, None

    def _scan_topics(self):
        topics = self.get_topic_names_and_types()
        for topic, types in topics:
            if topic in self._subs:
                continue

            cam, root, suffix = self._match_camera_topic(topic)
            if cam is None:
                continue

            # pick first type
            type_str = types[0] if types else None
            if not type_str:
                continue
            msg_cls = _msg_class_from_type(type_str)
            if msg_cls is None:
                self.get_logger().debug(f"Skipping unknown type {type_str} for {topic}")
                continue

            suffix = self._normalize_suffix(suffix)
            if not suffix:
                continue

            if suffix not in ALLOWED_SUFFIXES:
                continue

            out_topic = '/tf_static' if suffix == '/tf_static' else f"/{cam['namespace']}{suffix}"
            if out_topic == topic:
                # Avoid self-loop if topic is already in destination namespace.
                continue

            # default qos choices
            sub_qos_profile = QOS_TF_STATIC if topic.endswith('/tf_static') else qos_profile_sensor_data
            # Publish with best-effort to match camera driver QoS and avoid publisher disappearance
            pub_qos_profile = QOS_TF_STATIC if topic.endswith('/tf_static') else qos_profile_sensor_data

            # create publisher with chosen pub QoS
            pub = self.create_publisher(msg_cls, out_topic, pub_qos_profile)
            self._pubs[topic] = pub

            # create subscription with closure; count messages
            def make_cb(pub, cam, in_topic, dst_topic):
                def cb(msg):
                    try:
                        cnt = self._counters.setdefault(in_topic, {'in': 0, 'out': 0})
                        cnt['in'] += 1

                        if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'):
                            msg.header.frame_id = cam['frame']
                        pub.publish(msg)

                        cnt['out'] += 1
                    except Exception as e:
                        self.get_logger().error(
                            f"Failed to relay {in_topic} -> {dst_topic}: {e}"
                        )

                return cb

            sub = self.create_subscription(
                msg_cls,
                topic,
                make_cb(pub, cam, topic, out_topic),
                sub_qos_profile,
            )
            self._subs[topic] = sub
            self._seen_matches += 1
            self.get_logger().info(
                f"Relaying {topic} -> {out_topic} (type {type_str}, source root {root})"
            )

        if self._seen_matches == 0:
            self.get_logger().info(
                'No matching source topics found yet. Waiting for topics under '
                '/realsense/D555_<serial>... or /<namespace>/...'
            )

    def _report_counts(self):
        if not hasattr(self, '_counters') or not self._counters:
            return
        for topic, cnt in list(self._counters.items()):
            try:
                self.get_logger().info(f"Topic {topic} in={cnt.get('in',0)} out={cnt.get('out',0)}")
            except Exception:
                pass
            # reset counts for next interval
            self._counters[topic] = {'in': 0, 'out': 0}


def main(argv=None):
    argv = argv or sys.argv[1:]
    if len(argv) < 1:
        print("Usage: realsense_relay.py <config_path> [camera_flag]")
        return 2

    config_path = argv[0]
    camera_flag = argv[1] if len(argv) > 1 else 'enabled'

    rclpy.init()
    node = RealsenseRelay(config_path, camera_flag)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
