#!/usr/bin/env python3

# Replay a ROS 2 bag and reconstruct a PointCloud2 from recorded depth/colour topics.
#
# ── What to record ───────────────────────────────────────────────────────────
#
# Raw (simplest, largest files):
#   /<namespace>/<resolution>/camera_info
#   /<namespace>/<resolution>/image_color_rect
#   /<namespace>/<resolution>/image_depth_rect
#
# Compressed (recommended — smaller files, still lossless for depth):
#   /<namespace>/<resolution>/camera_info
#   /<namespace>/<resolution>/image_color_rect/compressed   ← JPEG, lossy colour only
#   /<namespace>/<resolution>/image_depth_rect/compressed   ← TIFF/PNG, lossless 16-bit
#
#   The kinect2_bridge encodes depth as TIFF (default) or PNG (use_png:=true),
#   both of which are lossless for uint16 data.  Colour JPEG loss only affects
#   RGB values painted onto points, not geometry.
#
# Video codecs (H.264 etc.) — DO NOT use for depth.
#   Standard video codecs target 8-bit YUV and will silently truncate/corrupt
#   the 16-bit mm depth values, destroying point geometry irreversibly.
#   Video encoding is fine for the colour image if geometry is all that matters.
#
# ── Compressed topic support ─────────────────────────────────────────────────
#
# If compressed topics were recorded, image_transport republisher nodes must be
# added here (before the pointcloud node) to decompress them back to raw
# sensor_msgs/Image, since depth_image_proc does not accept compressed inputs.
# See the TODO below in launch_setup().
#
# ─────────────────────────────────────────────────────────────────────────────
#
# Usage:
#   ros2 launch kinect2_bridge pointcloud_from_bag.launch.py bag:=/path/to/bag
#
# Optional arguments:
#   namespace:=kinect2        sensor namespace used when the bag was recorded
#   resolution:=qhd           qhd (960x540) or sd (512x424)
#   rate:=1.0                 playback rate multiplier
#   loop:=false               loop bag playback
#   launch_rviz:=true         open RViz to visualise the pointcloud
#
#
# eg SD resolution, half speed, looping, no RViz
# ros2 launch kinect2_bridge pointcloud_from_bag.launch.py \
#   bag:=/path/to/bag \
#   resolution:=sd \
#   rate:=0.5 \
#   loop:=true \
#   launch_rviz:=false

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    bag        = LaunchConfiguration("bag").perform(context)
    namespace  = LaunchConfiguration("namespace").perform(context)
    resolution = LaunchConfiguration("resolution").perform(context)
    rate       = LaunchConfiguration("rate").perform(context)
    loop       = LaunchConfiguration("loop").perform(context).lower() == "true"
    do_rviz    = LaunchConfiguration("launch_rviz").perform(context).lower() == "true"

    # ── Validate inputs ──────────────────────────────────────────────────── #

    if not bag:
        raise ValueError(
            "You must provide a bag path.\n"
            "  ros2 launch kinect2_bridge pointcloud_from_bag.launch.py bag:=/path/to/bag"
        )

    if not os.path.exists(bag):
        raise FileNotFoundError(f"Bag not found: {bag}")

    if resolution not in ("qhd", "sd"):
        raise ValueError(f"resolution must be 'qhd' or 'sd', got '{resolution}'")

    # ── Topic names (must match what was recorded) ────────────────────────── #

    camera_info  = f"/{namespace}/{resolution}/camera_info"
    color_rect   = f"/{namespace}/{resolution}/image_color_rect"
    depth_rect   = f"/{namespace}/{resolution}/image_depth_rect"
    points_out   = f"/{namespace}/{resolution}/points"

    # ── ros2 bag play ─────────────────────────────────────────────────────── #
    # --clock  re-publishes original timestamps → time-sync in depth_image_proc
    #          works correctly even at non-realtime rates
    # --topics limits playback to only the three topics we need (lighter I/O)

    play_cmd = [
        "ros2", "bag", "play", bag,
        "--clock",
        "--rate", rate,
        "--topics",
            camera_info,
            color_rect,
            depth_rect,
    ]

    if loop:
        play_cmd.append("--loop")

    bag_player = ExecuteProcess(
        cmd=play_cmd,
        output="screen",
    )

    # ── TODO: compressed topic decompression ───────────────────────────────── #
    # If the bag was recorded with compressed topics, add two
    # image_transport/republish nodes here — one for colour, one for depth —
    # to decompress back to raw sensor_msgs/Image before the pointcloud node.
    # depth_image_proc only accepts raw Image, not CompressedImage.
    #
    # Example (add once per compressed topic):
    #   Node(
    #       package='image_transport',
    #       executable='republish',
    #       name='decompress_depth',
    #       arguments=['compressed', 'raw'],
    #       remappings=[
    #           ('in/compressed', depth_rect + '/compressed'),
    #           ('out',           depth_rect),
    #       ],
    #       parameters=[{'use_sim_time': True}],
    #   )

    # ── depth_image_proc: PointCloud2 reconstruction ──────────────────────── #
    # Subscribes to the three topics above and publishes PointCloud2.
    # use_sim_time=True is required so the approximate time synchroniser
    # matches headers from the bag's /clock rather than wall clock.

    pointcloud_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        name=f"points_xyzrgb_{resolution}",
        namespace=namespace,
        output="screen",
        parameters=[{
            "queue_size": 10,
            "use_sim_time": True,
        }],
        remappings=[
            ("rgb/camera_info",            camera_info),
            ("rgb/image_rect_color",        color_rect),
            ("depth_registered/image_rect", depth_rect),
            ("points",                      points_out),
        ],
    )

    # ── Optional RViz ─────────────────────────────────────────────────────── #

    nodes = [bag_player, pointcloud_node]

    if do_rviz:
        pkg_share   = get_package_share_directory("kinect2_bridge")
        rviz_config = os.path.join(pkg_share, "launch", "kinect_viz.rviz")
        rviz_args   = ["-d", rviz_config] if os.path.isfile(rviz_config) else []

        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
            parameters=[{"use_sim_time": True}],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
            default_value="",
            description="Path to the ROS 2 bag directory to replay.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="kinect2",
            description="Sensor namespace used when the bag was recorded (e.g. kinect2, kinect2_1).",
        ),
        DeclareLaunchArgument(
            "resolution",
            default_value="qhd",
            description="Resolution to reconstruct: 'qhd' (960x540) or 'sd' (512x424).",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="1.0",
            description="Bag playback rate multiplier (e.g. 0.5 for half speed).",
        ),
        DeclareLaunchArgument(
            "loop",
            default_value="false",
            description="Loop bag playback indefinitely.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Open RViz to visualise the reconstructed pointcloud.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
