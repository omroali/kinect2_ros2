#!/usr/bin/env python3

# Dynamic multi-Kinect launch for ROS 2 Jazzy.
#
# Creates one Kinect pipeline per enabled entry in config/multi_camera_config.yaml.
# This file is the single source of truth for camera serials, frames and poses.
#
# Expected format:
#   world_frame: map
#   cameras:
#     kinect2_1:
#       serial: "001934470647"
#       enabled: true
#       frame: kinect2_1_link
#       position: {x: 0.0, y: 0.0, z: 1.0}
#       orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
#
# Camera transforms, serials and enable flags are read directly from the same
# config file.

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from kinect2_bridge.recording_config import resolve_config_path, selected_cameras


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

        entry = {
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
        }

        # Preserve point cloud filter config if present
        if "point_cloud_filter" in c and isinstance(c["point_cloud_filter"], dict):
            entry["point_cloud_filter"] = c["point_cloud_filter"]

        entries.append(entry)

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


def _has_pcl_passthrough_executable() -> bool:
    try:
        prefix = get_package_prefix("pcl_ros")
    except Exception:
        return False
    exec_path = os.path.join(prefix, "lib", "pcl_ros", "filter_passthrough_node")
    return os.path.isfile(exec_path) and os.access(exec_path, os.X_OK)


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("kinect2_bridge")
    config_path = LaunchConfiguration("config_file").perform(context)
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"camera config not found at {config_path}")

    world_frame, camera_entries = selected_cameras(config_path, "enabled")
    if not camera_entries:
        raise RuntimeError(f"No enabled cameras found in {config_path}")

    publish_tf = LaunchConfiguration("publish_transforms").perform(context).lower() in ("true", "1", "yes")
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in ("true", "1", "yes")
    delay_step = float(LaunchConfiguration("launch_delay_sec").perform(context))
    depth_method = LaunchConfiguration("depth_method").perform(context)
    pc_filter_min_dist = float(LaunchConfiguration("point_cloud_filter_min_distance").perform(context))
    pc_filter_max_dist = float(LaunchConfiguration("point_cloud_filter_max_distance").perform(context))

    pc_res_arg = LaunchConfiguration("point_cloud_resolution").perform(context)
    pc_res_list = [r.strip() for r in pc_res_arg.split(",") if r.strip() in ("sd", "qhd")]
    if not pc_res_list:
        pc_res_list = ["qhd"]
    has_passthrough_exec = _has_pcl_passthrough_executable()

    bridge_common = {
        "publish_tf": True,
        "fps_limit": 30.0,
        # PNG (lossless) for 16-bit depth on the /compressed topics; the
        # default (false) wraps depth in uncompressed TIFF. Encoding only
        # runs while something subscribes, i.e. during recording.
        "use_png": True,
        "png_level": 4,
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

            # Optional: Point cloud distance filter (depth-based crop for human activity recognition)
            # Subscribes to /{ns}/{res}/points and publishes /{ns}/{res}/points_filtered
            # Per-camera overrides can be set in multi_camera_config.yaml under camera.point_cloud_filter.
            filter_min = pc_filter_min_dist
            filter_max = pc_filter_max_dist
            if "point_cloud_filter" in c and isinstance(c["point_cloud_filter"], dict):
                filter_min = float(c["point_cloud_filter"].get("min_distance", pc_filter_min_dist))
                filter_max = float(c["point_cloud_filter"].get("max_distance", pc_filter_max_dist))

            if has_passthrough_exec:
                nodes.append(Node(
                    package="pcl_ros",
                    executable="filter_passthrough_node",
                    name=f"{ns}_points_filter_{res}",
                    namespace=ns,
                    output="screen",
                    parameters=[{
                        "filter_type": "passthrough",
                        "filter_field_name": "z",
                        "filter_limit_min": filter_min,
                        "filter_limit_max": filter_max,
                    }],
                    remappings=[
                        ("input", f"{res}/points"),
                        ("output", f"{res}/points_filtered"),
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
    pkg_share = get_package_share_directory("kinect2_bridge")
    default_config = resolve_config_path(
        "kinect_cameras.yaml",
        os.path.join(pkg_share, "config", "multi_camera_config.yaml"),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description=(
                "Unified camera YAML with serial+pose entries. Defaults to "
                "$SENSOR_CONFIG_DIR/kinect_cameras.yaml when available."
            ),
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
            default_value="default",
            description="libfreenect2 depth backend: opengl, opencl, cpu, default.",
        ),
        DeclareLaunchArgument(
            "point_cloud_filter_min_distance",
            default_value="0.0",
            description="Minimum distance (metres) for point cloud filter. Set to 0.0 to disable cropping at near end.",
        ),
        DeclareLaunchArgument(
            "point_cloud_filter_max_distance",
            default_value="5.0",
            description="Maximum distance (metres) for point cloud filter. Set to disable by using a large value.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
