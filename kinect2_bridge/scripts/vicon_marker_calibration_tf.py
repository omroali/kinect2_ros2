#!/usr/bin/env python3
"""
Vicon marker-based Kinect TF calibration.

This node estimates each camera pose from 4 Vicon marker topics (Top1..Top4),
then publishes map -> <camera_frame> transforms.

Per-camera marker geometry assumptions:
- Top1 is behind Top2 (camera forward axis points Top1 -> Top2)
- Marker labels Top3/Top4 are described from in front of the camera
    looking at it, so Top3 is observer-left and Top4 is observer-right.
    That means camera-left is Top4 and camera-right is Top3.
- Markers form an approximate square on the camera top plane

Configuration is read from multi_camera_config.yaml.
"""

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import yaml


Vec3 = Tuple[float, float, float]


def v_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v_norm(a: Vec3) -> float:
    return math.sqrt(v_dot(a, a))


def v_normalize(a: Vec3) -> Optional[Vec3]:
    n = v_norm(a)
    if n < 1e-9:
        return None
    return (a[0] / n, a[1] / n, a[2] / n)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_to_rpy(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    x, y, z, w = quat_normalize(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quat_angle_diff(q1: Tuple[float, float, float, float],
                    q2: Tuple[float, float, float, float]) -> float:
    a = quat_normalize(q1)
    b = quat_normalize(q2)
    dot = abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])
    dot = clamp(dot, -1.0, 1.0)
    return 2.0 * math.acos(dot)


def quat_slerp(q1: Tuple[float, float, float, float],
               q2: Tuple[float, float, float, float],
               alpha: float) -> Tuple[float, float, float, float]:
    a = clamp(alpha, 0.0, 1.0)
    q1n = quat_normalize(q1)
    q2n = quat_normalize(q2)

    dot = q1n[0] * q2n[0] + q1n[1] * q2n[1] + q1n[2] * q2n[2] + q1n[3] * q2n[3]
    if dot < 0.0:
        q2n = (-q2n[0], -q2n[1], -q2n[2], -q2n[3])
        dot = -dot

    if dot > 0.9995:
        out = (
            q1n[0] + a * (q2n[0] - q1n[0]),
            q1n[1] + a * (q2n[1] - q1n[1]),
            q1n[2] + a * (q2n[2] - q1n[2]),
            q1n[3] + a * (q2n[3] - q1n[3]),
        )
        return quat_normalize(out)

    theta_0 = math.acos(clamp(dot, -1.0, 1.0))
    sin_theta_0 = math.sin(theta_0)
    if abs(sin_theta_0) < 1e-9:
        return q1n

    theta = theta_0 * a
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    return (
        s0 * q1n[0] + s1 * q2n[0],
        s0 * q1n[1] + s1 * q2n[1],
        s0 * q1n[2] + s1 * q2n[2],
        s0 * q1n[3] + s1 * q2n[3],
    )


def rot_matrix_to_quat(r: List[List[float]]) -> Tuple[float, float, float, float]:
    # r is 3x3 row-major.
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif (r[0][0] > r[1][1]) and (r[0][0] > r[2][2]):
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s
    return quat_normalize((qx, qy, qz, qw))


def apply_rotation(r: List[List[float]], v: Vec3) -> Vec3:
    return (
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    )


def format_yaml_number(value: float) -> str:
    text = str(float(value))
    return "0.0" if text == "-0.0" else text


@dataclass
class CameraMarkerConfig:
    namespace: str
    frame: str
    vicon_object: str
    offset_xyz: Vec3


@dataclass
class CameraState:
    config: CameraMarkerConfig
    points: Dict[int, Vec3]
    stamps_sec: Dict[int, float]
    last_pose: Optional[Tuple[Vec3, Tuple[float, float, float, float]]] = None


class ViconMarkerCalibrationTF(Node):
    def __init__(self):
        super().__init__("vicon_marker_calibration_tf")

        self.declare_parameter("config_path", "")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("marker_topic_root", "/vicon/markers")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("max_marker_age_sec", 0.2)
        # Defaults are intentionally conservative for stable calibration visuals.
        self.declare_parameter("translation_alpha", 0.2)
        self.declare_parameter("rotation_alpha", 0.1)
        self.declare_parameter("translation_deadband_m", 0.003)
        self.declare_parameter("rotation_deadband_deg", 0.8)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.marker_topic_root = str(self.get_parameter("marker_topic_root").value).rstrip("/")
        self.max_marker_age_sec = float(self.get_parameter("max_marker_age_sec").value)
        self.translation_alpha = clamp(float(self.get_parameter("translation_alpha").value), 0.0, 1.0)
        self.rotation_alpha = clamp(float(self.get_parameter("rotation_alpha").value), 0.0, 1.0)
        self.translation_deadband_m = max(0.0, float(self.get_parameter("translation_deadband_m").value))
        self.rotation_deadband_rad = max(0.0, math.radians(float(self.get_parameter("rotation_deadband_deg").value)))

        config_path = str(self.get_parameter("config_path").value)
        if not config_path:
            raise RuntimeError("config_path parameter is required")
        if not os.path.isfile(config_path):
            raise FileNotFoundError(f"multi_camera_config file not found: {config_path}")
        self.config_path = config_path

        cameras = self._load_camera_marker_configs(config_path)
        if not cameras:
            raise RuntimeError("No enabled cameras found in multi_camera_config")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.cameras: Dict[str, CameraState] = {}
        self._subscriptions = []
        self._save_service = self.create_service(Trigger, "save_calibration", self._on_save_calibration)

        for cam in cameras:
            state = CameraState(config=cam, points={}, stamps_sec={})
            self.cameras[cam.namespace] = state
            for idx in (1, 2, 3, 4):
                topic = f"{self.marker_topic_root}/{cam.vicon_object}/Top{idx}"
                sub = self.create_subscription(
                    PointStamped,
                    topic,
                    lambda msg, ns=cam.namespace, i=idx: self._on_marker(ns, i, msg),
                    10,
                )
                self._subscriptions.append(sub)

        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_all)

        self.get_logger().info(
            f"Vicon calibration started for {len(self.cameras)} cameras. "
            f"Topic root: {self.marker_topic_root}, world frame: {self.world_frame}"
        )

    def _load_camera_marker_configs(self, config_path: str) -> List[CameraMarkerConfig]:
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f) or {}

        cameras_map = cfg.get("cameras", {})
        if not isinstance(cameras_map, dict):
            return []

        world_frame = str(cfg.get("world_frame", self.world_frame))
        self.world_frame = world_frame

        out: List[CameraMarkerConfig] = []
        for ns, c in cameras_map.items():
            if not isinstance(c, dict):
                continue
            if not bool(c.get("enabled", True)):
                continue

            frame = str(c.get("frame", f"{ns}_link"))

            cal = c.get("calibration", {}) if isinstance(c.get("calibration", {}), dict) else {}
            vicon_object = str(cal.get("vicon_object", ns))

            # Sensible default guess for Kinect2 link origin from marker-top centroid:
            # marker center is ~6.25 mm above top shell, shell half-height is 33 mm,
            # so marker plane is about +39.25 mm above link origin.
            # Offset from marker frame to link origin is therefore z = -0.03925 m.
            off = cal.get("marker_to_link_offset", {}) if isinstance(cal.get("marker_to_link_offset", {}), dict) else {}
            ox = float(off.get("x", 0.0))
            oy = float(off.get("y", 0.0))
            oz = float(off.get("z", -0.03925))

            out.append(CameraMarkerConfig(
                namespace=str(ns),
                frame=frame,
                vicon_object=vicon_object,
                offset_xyz=(ox, oy, oz),
            ))

        return out

    def _on_marker(self, ns: str, marker_idx: int, msg: PointStamped):
        state = self.cameras.get(ns)
        if state is None:
            return

        state.points[marker_idx] = (msg.point.x, msg.point.y, msg.point.z)
        state.stamps_sec[marker_idx] = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

    def _camera_pose_from_markers(self, state: CameraState) -> Optional[Tuple[Vec3, Tuple[float, float, float, float]]]:
        # Need all 4 square-defining markers.
        if not all(i in state.points for i in (1, 2, 3, 4)):
            return None

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        for i in (1, 2, 3, 4):
            age = now_sec - state.stamps_sec.get(i, 0.0)
            if age > self.max_marker_age_sec:
                return None

        p1 = state.points[1]
        p2 = state.points[2]
        p3 = state.points[3]
        p4 = state.points[4]

        center = v_scale(v_add(v_add(p1, p2), v_add(p3, p4)), 0.25)

        forward = v_normalize(v_sub(p2, p1))
        if forward is None:
            return None

        # Top3/Top4 were described from the observer's viewpoint in front of the camera.
        # That mirrors left/right relative to the camera's own frame, so camera-left is Top4.
        left_raw = v_sub(p4, p3)
        # Remove any forward component to keep orthogonality.
        left_proj = v_sub(left_raw, v_scale(forward, v_dot(left_raw, forward)))
        left = v_normalize(left_proj)
        if left is None:
            return None

        up = v_normalize(v_cross(forward, left))
        if up is None:
            return None

        # Recompute left for a strictly orthonormal basis.
        left = v_cross(up, forward)

        # Rotation matrix with camera axes as columns: X=forward, Y=left, Z=up.
        r = [
            [forward[0], left[0], up[0]],
            [forward[1], left[1], up[1]],
            [forward[2], left[2], up[2]],
        ]

        offset_world = apply_rotation(r, state.config.offset_xyz)
        translation = v_add(center, offset_world)
        quat = rot_matrix_to_quat(r)
        return translation, quat

    def _smooth_pose(self,
                     old_pose: Optional[Tuple[Vec3, Tuple[float, float, float, float]]],
                     new_pose: Tuple[Vec3, Tuple[float, float, float, float]]) -> Tuple[Vec3, Tuple[float, float, float, float]]:
        if old_pose is None:
            return new_pose

        old_t, old_q = old_pose
        new_t, new_q = new_pose

        t_alpha = self.translation_alpha
        q_alpha = self.rotation_alpha

        sm_t = (
            old_t[0] + t_alpha * (new_t[0] - old_t[0]),
            old_t[1] + t_alpha * (new_t[1] - old_t[1]),
            old_t[2] + t_alpha * (new_t[2] - old_t[2]),
        )
        sm_q = quat_slerp(old_q, new_q, q_alpha)
        return sm_t, sm_q

    def _publish_all(self):
        transforms: List[TransformStamped] = []

        for ns, state in self.cameras.items():
            pose = self._camera_pose_from_markers(state)
            if pose is None:
                continue

            if state.last_pose is not None:
                old_t, old_q = state.last_pose
                new_t, new_q = pose
                dx = new_t[0] - old_t[0]
                dy = new_t[1] - old_t[1]
                dz = new_t[2] - old_t[2]
                translation_delta = math.sqrt(dx * dx + dy * dy + dz * dz)
                rotation_delta = quat_angle_diff(old_q, new_q)

                if (
                    translation_delta < self.translation_deadband_m
                    and rotation_delta < self.rotation_deadband_rad
                ):
                    pose = state.last_pose

            pose = self._smooth_pose(state.last_pose, pose)
            state.last_pose = pose

            t, q = pose
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.world_frame
            msg.child_frame_id = state.config.frame
            msg.transform.translation.x = t[0]
            msg.transform.translation.y = t[1]
            msg.transform.translation.z = t[2]
            msg.transform.rotation.x = q[0]
            msg.transform.rotation.y = q[1]
            msg.transform.rotation.z = q[2]
            msg.transform.rotation.w = q[3]
            transforms.append(msg)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def _replace_section_values(self, block_lines: List[str], section_name: str, values: dict) -> List[str]:
        section_start = None
        for index, line in enumerate(block_lines):
            if line == f"    {section_name}:\n":
                section_start = index
                break

        if section_start is None:
            return block_lines

        section_end = section_start + 1
        while section_end < len(block_lines) and block_lines[section_end].startswith("      "):
            section_end += 1

        replacement_lines = [f"      {key}: {format_yaml_number(values[key])}\n" for key in values]
        return block_lines[:section_start + 1] + replacement_lines + block_lines[section_end:]

    def _update_camera_block(self, block_lines: List[str], pose: Tuple[Vec3, Tuple[float, float, float, float]]) -> List[str]:
        translation, quat = pose
        roll, pitch, yaw = quat_to_rpy(quat)

        position_values = {
            "x": translation[0],
            "y": translation[1],
            "z": translation[2],
        }
        orientation_values = {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }

        updated = self._replace_section_values(block_lines, "position", position_values)
        updated = self._replace_section_values(updated, "orientation", orientation_values)
        return updated

    def _find_camera_block(self, lines: List[str], camera_key: str) -> Optional[Tuple[int, int]]:
        start = None
        target = f"  {camera_key}:\n"
        for index, line in enumerate(lines):
            if line == target:
                start = index
                break

        if start is None:
            return None

        end = len(lines)
        for index in range(start + 1, len(lines)):
            line = lines[index]
            if line.startswith("  ") and not line.startswith("    ") and line.strip().endswith(":") and not line.lstrip().startswith("#"):
                end = index
                break

        return start, end

    def _on_save_calibration(self, request, response):
        del request

        with open(self.config_path, "r") as f:
            lines = f.readlines()

        updated = []
        missing = []
        block_updates = []

        for ns, state in self.cameras.items():
            pose = self._camera_pose_from_markers(state)
            if pose is None:
                missing.append(ns)
                continue

            block = self._find_camera_block(lines, state.config.namespace)
            if block is None:
                missing.append(ns)
                continue

            start, end = block
            block_updates.append((start, end, pose))
            updated.append(ns)

        if not updated:
            response.success = False
            response.message = "No camera poses were saved; calibration data is not ready yet"
            return response

        for start, end, pose in sorted(block_updates, key=lambda item: item[0], reverse=True):
            block_lines = lines[start:end]
            updated_block = self._update_camera_block(block_lines, pose)
            lines[start:end] = updated_block

        tmp_path = f"{self.config_path}.tmp"
        with open(tmp_path, "w") as f:
            f.writelines(lines)
        os.replace(tmp_path, self.config_path)

        response.success = True
        response.message = (
            f"Saved calibration for {', '.join(updated)} to {self.config_path}. "
            + (f"Missing or stale: {', '.join(missing)}" if missing else "")
        ).strip()
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ViconMarkerCalibrationTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
