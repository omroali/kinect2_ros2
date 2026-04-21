#!/usr/bin/env python3

# Dynamic multi-Kinect launch for ROS 2 Jazzy.
#
# Creates one Kinect pipeline per serial entry in config/dual_kinect_serials.yaml
# (or another file passed via serials_config:=...).
#
# Preferred unified format (single file):
#   world_frame: map
#   cameras:
#     kinect2_1:
#       serial: "001934470647"
#       enabled: true
#       frame: kinect2_1_link
#       position: {x: 0.0, y: 0.0, z: 1.0}
#       orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
#
# Supported serials YAML format (same as current dual file):
#   /kinect2_1/kinect2_bridge:
#     ros__parameters:
#       sensor: "001934470647"
#
# Camera transform config supports either:
# 1) Legacy keys camera1/camera2/... with position/orientation/frame, matched by order
# 2) cameras:<namespace>:... keyed directly by namespace

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_serial_entries(serials_path: str):
    with open(serials_path, "r") as f:
        data = yaml.safe_load(f) or {}

    entries = []
    for key, val in data.items():
        ns = key.strip("/").split("/")[0]
        serial = str(val.get("ros__parameters", {}).get("sensor", "")).strip()
        if serial:
            entries.append((ns, serial))
    return entries


def _load_camera_config(config_path: str) -> dict:
    if not os.path.isfile(config_path):
        return {}
    with open(config_path, "r") as f:
        return yaml.safe_load(f) or {}


def _load_unified_config(unified_path: str):
    if not os.path.isfile(unified_path):
        return "map", []

    with open(unified_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    cameras = cfg.get("cameras", {})
    if not isinstance(cameras, dict):
        return str(cfg.get("world_frame", "map")), []

    world_frame = str(cfg.get("world_frame", "map"))
    entries = []

    for ns, c in cameras.items():
        if not isinstance(c, dict):
            continue
        serial = str(c.get("serial", "")).strip()
        if not serial:
            continue

        pos = c.get("position", {}) if isinstance(c.get("position", {}), dict) else {}
        ori = c.get("orientation", {}) if isinstance(c.get("orientation", {}), dict) else {}

        entries.append({
            "namespace": str(ns),
            "serial": serial,
            "enabled": bool(c.get("enabled", True)),
            "frame": str(c.get("frame", f"{ns}_link")),
            "x": str(float(pos.get("x", 0.0))),
            "y": str(float(pos.get("y", 0.0))),
            "z": str(float(pos.get("z", 0.0))),
            "roll": str(float(ori.get("roll", 0.0))),
            "pitch": str(float(ori.get("pitch", 0.0))),
            "yaw": str(float(ori.get("yaw", 0.0))),
        })

    return world_frame, entries


def _camera_cfg_for(ns: str, index: int, cam_cfg: dict) -> dict:
    # Prefer namespace-keyed config: cameras: {kinect2_1: {...}}
    cameras_map = cam_cfg.get("cameras", {}) if isinstance(cam_cfg.get("cameras", {}), dict) else {}
    if ns in cameras_map and isinstance(cameras_map[ns], dict):
        cfg = cameras_map[ns]
    else:
        # Fallback to legacy camera1/camera2/... keys by serial list order.
        cfg = cam_cfg.get(f"camera{index + 1}", {})

    pos = cfg.get("position", {}) if isinstance(cfg, dict) else {}
    ori = cfg.get("orientation", {}) if isinstance(cfg, dict) else {}

    return {
        "enabled": bool(cfg.get("enabled", True)) if isinstance(cfg, dict) else True,
        "frame": str(cfg.get("frame", f"{ns}_link")) if isinstance(cfg, dict) else f"{ns}_link",
        "x": str(float(pos.get("x", 0.0))),
        "y": str(float(pos.get("y", 0.0))),
        "z": str(float(pos.get("z", 0.0))),
        "roll": str(float(ori.get("roll", 0.0))),
        "pitch": str(float(ori.get("pitch", 0.0))),
        "yaw": str(float(ori.get("yaw", 0.0))),
    }


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("kinect2_bridge")
    config_dir = os.path.join(pkg_share, "config")

    unified_rel = LaunchConfiguration("multi_camera_config").perform(context)
    serials_rel = LaunchConfiguration("serials_config").perform(context)
    camera_rel = LaunchConfiguration("camera_config").perform(context)

    unified_path = unified_rel if os.path.isabs(unified_rel) else os.path.join(config_dir, unified_rel)
    serials_path = serials_rel if os.path.isabs(serials_rel) else os.path.join(config_dir, serials_rel)
    camera_path = camera_rel if os.path.isabs(camera_rel) else os.path.join(config_dir, camera_rel)

    # Prefer the unified config when available; fallback to legacy 2-file format.
    world_frame, unified_entries = _load_unified_config(unified_path)
    use_unified = len(unified_entries) > 0

    if not use_unified:
        if not os.path.isfile(serials_path):
            raise FileNotFoundError(
                f"No valid unified config at {unified_path} and serial config not found: {serials_path}"
            )

        serial_entries = _load_serial_entries(serials_path)
        if not serial_entries:
            raise RuntimeError(
                f"No valid camera entries in unified config ({unified_path}) and no serial entries in {serials_path}"
            )

    publish_tf = LaunchConfiguration("publish_transforms").perform(context).lower() in ("true", "1", "yes")
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in ("true", "1", "yes")
    delay_step = float(LaunchConfiguration("launch_delay_sec").perform(context))
    depth_method = LaunchConfiguration("depth_method").perform(context)

    pc_res_arg = LaunchConfiguration("point_cloud_resolution").perform(context)
    pc_res_list = [r.strip() for r in pc_res_arg.split(",") if r.strip() in ("sd", "qhd")]
    if not pc_res_list:
        pc_res_list = ["qhd"]

    cam_cfg = {} if use_unified else _load_camera_config(camera_path)
    if not use_unified:
        world_frame = str(cam_cfg.get("world_frame", "map"))

    bridge_common = {
        "publish_tf": True,
        "fps_limit": 30.0,
        "use_png": False,
        "depth_method": depth_method,
        "reg_method": "default",
        "max_depth": 12.0,
        "min_depth": 0.1,
        "queue_size": 5,
        "bilateral_filter": True,
        "edge_aware_filter": True,
        "worker_threads": 4,
    }

    nodes = []

    camera_entries = unified_entries if use_unified else []
    if not use_unified:
        for i, (ns, serial) in enumerate(serial_entries):
            c = _camera_cfg_for(ns, i, cam_cfg)
            camera_entries.append({
                "namespace": ns,
                "serial": serial,
                "enabled": c["enabled"],
                "frame": c["frame"],
                "x": c["x"],
                "y": c["y"],
                "z": c["z"],
                "roll": c["roll"],
                "pitch": c["pitch"],
                "yaw": c["yaw"],
            })

    for i, c in enumerate(camera_entries):
        ns = c["namespace"]
        serial = c["serial"]
        bridge = Node(
            package="kinect2_bridge",
            executable="kinect2_bridge_node",
            name=f"{ns}_bridge",
            output="screen",
            ros_arguments=["-p", f"sensor:='{serial}'"],
            parameters=[{
                **bridge_common,
                "base_name": ns,
                "base_name_tf": ns,
            }],
        )

        if i == 0 or delay_step <= 0.0:
            nodes.append(bridge)
        else:
            nodes.append(TimerAction(period=i * delay_step, actions=[bridge]))

        for res in pc_res_list:
            nodes.append(Node(
                package="depth_image_proc",
                executable="point_cloud_xyzrgb_node",
                name=f"{ns}_points_xyzrgb_{res}",
                namespace=ns,
                output="screen",
                parameters=[{"queue_size": 10}],
                remappings=[
                    ("rgb/camera_info", f"/{ns}/{res}/camera_info"),
                    ("rgb/image_rect_color", f"/{ns}/{res}/image_color_rect"),
                    ("depth_registered/image_rect", f"/{ns}/{res}/image_depth_rect"),
                    ("points", f"{res}/points"),
                ],
            ))

        if publish_tf and c["enabled"]:
                nodes.append(Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"{ns}_map_tf",
                    output="screen",
                    arguments=[
                        "--frame-id", world_frame,
                        "--child-frame-id", c["frame"],
                        "--x", c["x"],
                        "--y", c["y"],
                        "--z", c["z"],
                        "--roll", c["roll"],
                        "--pitch", c["pitch"],
                        "--yaw", c["yaw"],
                    ],
                ))

    if launch_rviz:
        rviz_config = os.path.join(pkg_share, "launch", "kinect_viz.rviz")
        rviz_args = ["-d", rviz_config] if os.path.isfile(rviz_config) else []
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "multi_camera_config",
            default_value="multi_camera_config.yaml",
            description=(
                "Unified camera YAML with serial+pose entries. "
                "If missing/empty, launcher falls back to serials_config + camera_config."
            ),
        ),
        DeclareLaunchArgument(
            "serials_config",
            default_value="dual_kinect_serials.yaml",
            description="Serial mapping YAML in package config dir (or absolute path).",
        ),
        DeclareLaunchArgument(
            "camera_config",
            default_value="camera_config.yaml",
            description="Camera pose YAML in package config dir (or absolute path).",
        ),
        DeclareLaunchArgument(
            "publish_transforms",
            default_value="true",
            description="Publish map->camera_link static transforms for each configured camera.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz after sensor nodes.",
        ),
        DeclareLaunchArgument(
            "point_cloud_resolution",
            default_value="qhd",
            description="Comma-separated: qhd, sd, or qhd,sd.",
        ),
        DeclareLaunchArgument(
            "launch_delay_sec",
            default_value="5.0",
            description="Delay step in seconds between camera bridge startups.",
        ),
        DeclareLaunchArgument(
            "depth_method",
            default_value="opengl",
            description="libfreenect2 depth backend: opengl, opencl, cpu, default.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
