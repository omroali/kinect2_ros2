#!/usr/bin/env python3

# Copyright (c) 2024
# Dual Kinect v2 Launch File for ROS 2
#
# Loads configuration from:
#   - config/dual_kinect_serials.yaml  (serial numbers per camera namespace)
#   - config/camera_config.yaml        (camera positions and orientations)
#
# Topic architecture (why bridge nodes run at ROOT namespace):
# ============================================================
# The kinect2_bridge code constructs publisher topics as:
#
#     create_publisher(base_name + "/qhd/image_color_rect", ...)
#
# base_name = "kinect2_1", no ROS namespace:
#   relative topic "kinect2_1/qhd/image_color_rect"
#   → resolves to /kinect2_1/qhd/image_color_rect  ✓
#
# base_name = "kinect2_1", namespace = "/kinect2_1"  ← OLD BROKEN APPROACH:
#   relative topic "kinect2_1/qhd/image_color_rect" in namespace "/kinect2_1"
#   → resolves to /kinect2_1/kinect2_1/qhd/image_color_rect  ✗  (doubled!)
#   → depth_image_proc never receives images → device never streams → empty topics
#
# Fix: bridge nodes have NO namespace; depth_image_proc uses absolute remappings.
#
# TF chain (set Fixed Frame = "map" in RViz):
#   map
#   ├── kinect2_1_link          ← static_transform_publisher (camera_config.yaml)
#   │   └── kinect2_1_rgb_optical_frame   ← bridge publishStaticTF()
#   │       └── kinect2_1_ir_optical_frame
#   └── kinect2_2_link
#       └── kinect2_2_rgb_optical_frame
#           └── kinect2_2_ir_optical_frame
#
# Usage:
#   ros2 launch kinect2_bridge dual_kinect2_simple.launch.py
#
# Overrides:
#   ros2 launch kinect2_bridge dual_kinect2_simple.launch.py \
#       camera1_namespace:=kinect2_1 \
#       camera2_namespace:=kinect2_2 \
#       publish_transforms:=false

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────────── #
#  Config helpers                                                       #
# ─────────────────────────────────────────────────────────────────── #

def _load_serials(serials_path: str) -> dict:
    """
    Parse dual_kinect_serials.yaml → {namespace: serial_string}.

    Expected YAML format:
        /kinect2_1/kinect2_bridge:
          ros__parameters:
            sensor: "001934470647"
        /kinect2_2/kinect2_bridge:
          ros__parameters:
            sensor: "007425354147"
    """
    with open(serials_path, "r") as f:
        data = yaml.safe_load(f)

    result = {}
    for key, val in data.items():
        # key is like '/kinect2_1/kinect2_bridge' — take the first path component
        ns = key.strip("/").split("/")[0]
        result[ns] = str(val["ros__parameters"]["sensor"])
    return result


def _load_camera_config(config_path: str) -> dict:
    """
    Parse camera_config.yaml → flat dict of string values for
    use as static_transform_publisher CLI arguments.
    """
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    def _s(val):
        """Numeric → plain string without scientific notation."""
        return str(float(val))

    return {
        "world_frame": cfg.get("world_frame", "map"),
        # Camera 1
        "cam1_frame":  cfg["camera1"]["frame"],
        "cam1_x":      _s(cfg["camera1"]["position"]["x"]),
        "cam1_y":      _s(cfg["camera1"]["position"]["y"]),
        "cam1_z":      _s(cfg["camera1"]["position"]["z"]),
        "cam1_roll":   _s(cfg["camera1"]["orientation"]["roll"]),
        "cam1_pitch":  _s(cfg["camera1"]["orientation"]["pitch"]),
        "cam1_yaw":    _s(cfg["camera1"]["orientation"]["yaw"]),
        # Camera 2
        "cam2_frame":  cfg["camera2"]["frame"],
        "cam2_x":      _s(cfg["camera2"]["position"]["x"]),
        "cam2_y":      _s(cfg["camera2"]["position"]["y"]),
        "cam2_z":      _s(cfg["camera2"]["position"]["z"]),
        "cam2_roll":   _s(cfg["camera2"]["orientation"]["roll"]),
        "cam2_pitch":  _s(cfg["camera2"]["orientation"]["pitch"]),
        "cam2_yaw":    _s(cfg["camera2"]["orientation"]["yaw"]),
    }


# ─────────────────────────────────────────────────────────────────── #
#  OpaqueFunction                                                       #
#  All node construction happens here so that LaunchConfiguration      #
#  values are resolved to plain Python strings before being used in    #
#  f-strings for absolute topic paths.                                 #
# ─────────────────────────────────────────────────────────────────── #

def launch_setup(context, *args, **kwargs):
    # Resolve launch arguments to Python strings
    ns1       = LaunchConfiguration("camera1_namespace").perform(context)
    ns2       = LaunchConfiguration("camera2_namespace").perform(context)
    do_tf_str = LaunchConfiguration("publish_transforms").perform(context)
    do_tf     = do_tf_str.lower() in ("true", "1", "yes")

    # ── Locate installed config files ──────────────────────────────── #
    pkg_share  = get_package_share_directory("kinect2_bridge")
    config_dir = os.path.join(pkg_share, "config")

    serials_path    = os.path.join(config_dir, "dual_kinect_serials.yaml")
    cam_config_path = os.path.join(config_dir, "camera_config.yaml")

    if not os.path.isfile(serials_path):
        raise FileNotFoundError(
            f"Serial config not found: {serials_path}\n"
            "Did you rebuild with 'colcon build --packages-select kinect2_bridge'?"
        )
    if not os.path.isfile(cam_config_path):
        raise FileNotFoundError(
            f"Camera config not found: {cam_config_path}\n"
            "Did you rebuild with 'colcon build --packages-select kinect2_bridge'?"
        )

    serials = _load_serials(serials_path)
    cam     = _load_camera_config(cam_config_path)

    serial1 = serials.get(ns1, "")
    serial2 = serials.get(ns2, "")

    nodes = []

    # ── Bridge nodes ───────────────────────────────────────────────── #
    #
    # Run at ROOT namespace (no namespace= argument).
    #
    # The bridge creates publishers as:
    #   create_publisher(base_name + "/qhd/image_color_rect", ...)
    # With base_name="kinect2_1" and no ROS namespace, the relative
    # topic resolves to the absolute path /kinect2_1/qhd/image_color_rect.
    #
    # base_name_tf = ns  →  bridge appends "_link" internally, producing
    # the TF root frame  kinect2_N_link, which our static publishers
    # connect to the world frame.
    #
    _bridge_common = dict(
        publish_tf=True,
        fps_limit=30.0,
        use_png=False,
        depth_method="default",
        reg_method="default",
        max_depth=12.0,
        min_depth=0.1,
        queue_size=5,
        bilateral_filter=True,
        edge_aware_filter=True,
        worker_threads=4,
    )

    # sensor is passed via ros_arguments with explicit YAML single-quote wrapping
    # (e.g. sensor:='007425354147') so the ROS 2 parameter loader always treats
    # the value as a string.  If passed through the parameters dict, launch_ros
    # may serialise a numeric-looking serial to the temp YAML file without quotes,
    # causing rcl_yaml_param_parser to infer a numeric type and throw
    # InvalidParameterTypeException when the C++ node declares it as std::string.
    nodes.append(Node(
        package="kinect2_bridge",
        executable="kinect2_bridge_node",
        name=f"{ns1}_bridge",
        output="screen",
        ros_arguments=['-p', f"sensor:='{serial1}'"],
        parameters=[{
            **_bridge_common,
            "base_name":    ns1,      # topics → /kinect2_1/hd/, /qhd/, /sd/
            "base_name_tf": ns1,      # TF root = kinect2_1_link
        }],
    ))

    # ── Delay second bridge by 5 s ────────────────────────────────── #
    #
    # libfreenect2 calls device->start() then device->stop() during
    # initDevice() just to read camera parameters.  When both bridges
    # race through that sequence simultaneously on the same USB bus one
    # of them fails silently and the node shuts down without ever
    # creating its publishers.  A 5-second gap is enough for the first
    # device to finish its USB init/start/stop cycle before the second
    # one begins.
    #
    # sensor is passed via ros_arguments (see bridge1 comment above).
    #
    nodes.append(TimerAction(
        period=5.0,
        actions=[Node(
            package="kinect2_bridge",
            executable="kinect2_bridge_node",
            name=f"{ns2}_bridge",
            output="screen",
            ros_arguments=['-p', f"sensor:='{serial2}'"],
            parameters=[{
                **_bridge_common,
                "base_name":    ns2,
                "base_name_tf": ns2,
            }],
        )],
    ))

    # ── Point cloud nodes ──────────────────────────────────────────── #
    #
    # Each depth_image_proc node is given an explicit namespace so its
    # output topic resolves to  /<ns>/<res>/points.
    #
    # Input remappings use ABSOLUTE paths (leading /) to reach the
    # bridge topics that live at the root namespace regardless of this
    # node's own namespace.
    #
    # SD  resolution  512 × 424  — depth-registered colour + depth rect
    # QHD resolution  960 × 540  — same at quarter-HD scale
    #
    for ns in (ns1, ns2):
        for res in ("sd", "qhd"):
            nodes.append(Node(
                package="depth_image_proc",
                executable="point_cloud_xyzrgb_node",
                name=f"points_xyzrgb_{res}",
                namespace=ns,
                output="screen",
                parameters=[{"queue_size": 10}],
                remappings=[
                    # Absolute → bridge's topic at /<ns>/<res>/...
                    ("rgb/camera_info",             f"/{ns}/{res}/camera_info"),
                    ("rgb/image_rect_color",         f"/{ns}/{res}/image_color_rect"),
                    ("depth_registered/image_rect",  f"/{ns}/{res}/image_depth_rect"),
                    # Relative output → /<ns>/<res>/points
                    ("points",                       f"{res}/points"),
                ],
            ))

    # ── Static TF publishers ───────────────────────────────────────── #
    #
    # Publishes:
    #   map → kinect2_1_link   (positions from camera_config.yaml)
    #   map → kinect2_2_link
    #
    # The bridge's publishStaticTF() then extends the tree:
    #   kinect2_1_link → kinect2_1_rgb_optical_frame
    #   kinect2_1_rgb_optical_frame → kinect2_1_ir_optical_frame
    #
    if do_tf:
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera1_tf_publisher",
            arguments=[
                "--frame-id",       cam["world_frame"],
                "--child-frame-id", cam["cam1_frame"],
                "--x",              cam["cam1_x"],
                "--y",              cam["cam1_y"],
                "--z",              cam["cam1_z"],
                "--roll",           cam["cam1_roll"],
                "--pitch",          cam["cam1_pitch"],
                "--yaw",            cam["cam1_yaw"],
            ],
        ))

        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera2_tf_publisher",
            arguments=[
                "--frame-id",       cam["world_frame"],
                "--child-frame-id", cam["cam2_frame"],
                "--x",              cam["cam2_x"],
                "--y",              cam["cam2_y"],
                "--z",              cam["cam2_z"],
                "--roll",           cam["cam2_roll"],
                "--pitch",          cam["cam2_pitch"],
                "--yaw",            cam["cam2_yaw"],
            ],
        ))

    return nodes


# ─────────────────────────────────────────────────────────────────── #
#  Entry point                                                          #
# ─────────────────────────────────────────────────────────────────── #

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera1_namespace",
            default_value="kinect2_1",
            description=(
                "Namespace and base_name for the first Kinect v2. "
                "Topics land at /<namespace>/hd/, /qhd/, /sd/. "
                "Serial is looked up by this key in dual_kinect_serials.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "camera2_namespace",
            default_value="kinect2_2",
            description=(
                "Namespace and base_name for the second Kinect v2. "
                "Topics land at /<namespace>/hd/, /qhd/, /sd/. "
                "Serial is looked up by this key in dual_kinect_serials.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "publish_transforms",
            default_value="true",
            description=(
                "Publish static TF transforms from world_frame (map) to each "
                "camera link frame. Positions are read from camera_config.yaml. "
                "Set to 'false' if you provide transforms externally."
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
