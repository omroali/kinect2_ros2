#!/usr/bin/env python3

# Copyright (c) 2024
# Kinect v2 Sensor Visualisation Launch File
#
# Reads camera_config.yaml and generates a URDF that renders each Kinect v2
# as a labelled 3-D box model in RViz at the exact pose defined in the config.
#
# The URDF contains fixed joints (map → kinect2_N_link), so robot_state_publisher
# also broadcasts those TF transforms.  When run alongside
# dual_kinect2_simple.launch.py the transforms are published twice with identical
# values — TF2 handles this gracefully.
#
# Usage (standalone — layout planning, no live cameras needed):
#   ros2 launch kinect2_bridge kinect_viz.launch.py
#
# Usage (overlay — add sensor models to a live capture session):
#   Terminal 1:  ros2 launch kinect2_bridge dual_kinect2_simple.launch.py
#   Terminal 2:  ros2 launch kinect2_bridge kinect_viz.launch.py launch_rviz:=false
#
# Arguments:
#   launch_rviz   true/false   Open RViz automatically (default: true)
#   rviz_config   path         RViz .rviz config file   (default: package kinect_viz.rviz)

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────────────────── #
#  Kinect v2 physical dimensions (metres)                                      #
# ─────────────────────────────────────────────────────────────────────────── #
#  Real sensor: 249 mm wide × 67 mm deep × 66 mm tall
#  The "face" of the sensor (where the lenses sit) is on the +X side of the
#  link frame, consistent with the ROS camera convention (+X forward).
#
KV2_W = 0.249   # width  — Y axis (left / right)
KV2_H = 0.066   # height — Z axis (up / down)
KV2_D = 0.067   # depth  — X axis (forward / back, face = +X)

# Colours (RGBA)
COL_BODY   = "0.13 0.13 0.13 1.0"   # near-black body
COL_FACE   = "0.22 0.22 0.22 1.0"   # slightly lighter front face
COL_LENS   = "0.05 0.05 0.08 1.0"   # very dark blue-black lens windows
COL_STRIPE = "0.60 0.60 0.60 1.0"   # light grey top-label stripe

# Camera accent colours so the two sensors are visually distinct in RViz
CAM_ACCENTS = [
    "0.20 0.45 0.80 1.0",   # camera1 — blue
    "0.80 0.35 0.15 1.0",   # camera2 — orange
    "0.20 0.70 0.30 1.0",   # camera3 — green  (future expansion)
    "0.75 0.20 0.70 1.0",   # camera4 — purple
]


# ─────────────────────────────────────────────────────────────────────────── #
#  URDF helpers                                                                #
# ─────────────────────────────────────────────────────────────────────────── #

def _box(sx, sy, sz):
    return f'<geometry><box size="{sx:.4f} {sy:.4f} {sz:.4f}"/></geometry>'


def _visual(name, ox, oy, oz, roll, pitch, yaw, geometry_xml, color_rgba):
    return f"""
        <visual name="{name}">
          <origin xyz="{ox:.4f} {oy:.4f} {oz:.4f}" rpy="{roll:.4f} {pitch:.4f} {yaw:.4f}"/>
          {geometry_xml}
          <material name="{name}_mat">
            <color rgba="{color_rgba}"/>
          </material>
        </visual>"""


def _kinect_link(link_name: str, accent_color: str) -> str:
    """Return URDF <link> XML for one Kinect v2 sensor.

    Visual elements (all in the link frame, face pointing +X):
      1. Main body              — full-size dark box
      2. Front face panel       — thin slab on +X face, slightly lighter
      3. Top label stripe       — narrow strip on the top-rear edge
      4. IR emitter window      — small square, offset to left of face
      5. RGB camera window      — small square, centre of face
      6. Depth sensor window    — small square, offset to right of face
      7. Accent strip           — thin coloured strip along the bottom edge
         (colour unique per camera so they are easy to tell apart in RViz)
    """

    face_x   = KV2_D / 2          # X position of the front face centre
    face_slab = 0.003              # thickness of the face panel

    # Lens / window dimensions
    lens_w, lens_h = 0.028, 0.028

    # Lateral positions of the three windows along Y (left → right looking from front)
    ir_y     =  KV2_W * 0.33      # IR emitter  — left
    rgb_y    =  0.000             # RGB camera  — centre
    depth_y  = -KV2_W * 0.33     # Depth cam   — right

    # Top stripe
    stripe_h     = 0.008
    stripe_z_off = KV2_H / 2 - stripe_h / 2

    # Bottom accent strip
    accent_h     = 0.006
    accent_z_off = -(KV2_H / 2 - accent_h / 2)

    visuals = ""

    # 1. Main body
    visuals += _visual(
        f"{link_name}_body",
        0, 0, 0, 0, 0, 0,
        _box(KV2_D, KV2_W, KV2_H),
        COL_BODY,
    )

    # 2. Front face panel
    visuals += _visual(
        f"{link_name}_face",
        face_x + face_slab / 2, 0, 0, 0, 0, 0,
        _box(face_slab, KV2_W, KV2_H),
        COL_FACE,
    )

    # 3. Top label stripe (sits on body top)
    visuals += _visual(
        f"{link_name}_stripe",
        0, 0, stripe_z_off, 0, 0, 0,
        _box(KV2_D * 0.85, KV2_W * 0.92, stripe_h),
        COL_STRIPE,
    )

    # 4. IR emitter window
    visuals += _visual(
        f"{link_name}_ir",
        face_x + face_slab + 0.001, ir_y, 0, 0, 0, 0,
        _box(0.002, lens_w, lens_h),
        COL_LENS,
    )

    # 5. RGB camera window
    visuals += _visual(
        f"{link_name}_rgb",
        face_x + face_slab + 0.001, rgb_y, 0, 0, 0, 0,
        _box(0.002, lens_w, lens_h),
        COL_LENS,
    )

    # 6. Depth sensor window
    visuals += _visual(
        f"{link_name}_depth",
        face_x + face_slab + 0.001, depth_y, 0, 0, 0, 0,
        _box(0.002, lens_w, lens_h),
        COL_LENS,
    )

    # 7. Accent strip along the bottom front edge
    visuals += _visual(
        f"{link_name}_accent",
        face_x - 0.005, 0, accent_z_off, 0, 0, 0,
        _box(0.010, KV2_W * 0.95, accent_h),
        accent_color,
    )

    return f'  <link name="{link_name}">{visuals}\n  </link>'


def _fixed_joint(joint_name, parent, child, xyz, rpy):
    x, y, z    = xyz
    ro, pi, ya = rpy
    return f"""  <joint name="{joint_name}" type="fixed">
    <parent link="{parent}"/>
    <child  link="{child}"/>
    <origin xyz="{x:.6f} {y:.6f} {z:.6f}" rpy="{ro:.6f} {pi:.6f} {ya:.6f}"/>
  </joint>"""


# ─────────────────────────────────────────────────────────────────────────── #
#  Config loader                                                               #
# ─────────────────────────────────────────────────────────────────────────── #

def _load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


# ─────────────────────────────────────────────────────────────────────────── #
#  URDF generator                                                              #
# ─────────────────────────────────────────────────────────────────────────── #

def _generate_urdf(cfg: dict) -> str:
    """Build a complete URDF string from camera_config.yaml content.

    Structure
    ---------
    <robot name="kinect2_cameras">
      <link name="map"/>                           ← world anchor

      <link name="kinect2_1_link"> ... </link>     ← sensor 1 visuals
      <joint ...>                                  ← map → kinect2_1_link
        <origin xyz="3.72 1.18 0.706" rpy="0 0 0"/>
      </joint>

      <link name="kinect2_2_link"> ... </link>     ← sensor 2 visuals
      <joint ...>                                  ← map → kinect2_2_link
        <origin xyz="3.72 1.18 0.706" rpy="0 0 -0.5236"/>
      </joint>
    </robot>
    """
    world_frame = cfg.get("world_frame", "map")

    parts = [
        '<?xml version="1.0"?>',
        '<robot name="kinect2_cameras">',
        f'  <!-- World anchor -->',
        f'  <link name="{world_frame}"/>',
        '',
    ]

    camera_index = 0
    for i in range(1, 20):
        key = f"camera{i}"
        if key not in cfg:
            break
        cam = cfg[key]
        if not cam.get("enabled", True):
            continue

        frame = cam["frame"]
        pos   = cam["position"]
        ori   = cam["orientation"]
        xyz   = (float(pos["x"]), float(pos["y"]), float(pos["z"]))
        rpy   = (
            float(ori.get("roll",  0.0)),
            float(ori.get("pitch", 0.0)),
            float(ori.get("yaw",   0.0)),
        )
        accent = CAM_ACCENTS[camera_index % len(CAM_ACCENTS)]

        parts.append(f'  <!-- ── Camera {i}: {frame} ────────────────────────────── -->')
        parts.append(_kinect_link(frame, accent))
        parts.append('')
        parts.append(_fixed_joint(
            joint_name=f"{world_frame}_to_{frame}",
            parent=world_frame,
            child=frame,
            xyz=xyz,
            rpy=rpy,
        ))
        parts.append('')

        camera_index += 1

    parts.append('</robot>')
    return '\n'.join(parts)


# ─────────────────────────────────────────────────────────────────────────── #
#  OpaqueFunction — all runtime logic lives here                               #
# ─────────────────────────────────────────────────────────────────────────── #

def launch_setup(context, *args, **kwargs):
    pkg_share   = get_package_share_directory("kinect2_bridge")
    config_path = os.path.join(pkg_share, "config", "camera_config.yaml")

    if not os.path.isfile(config_path):
        raise FileNotFoundError(
            f"camera_config.yaml not found at {config_path}\n"
            "Run 'colcon build --packages-select kinect2_bridge --symlink-install'."
        )

    cfg  = _load_config(config_path)
    urdf = _generate_urdf(cfg)

    do_rviz     = LaunchConfiguration("launch_rviz").perform(context).lower() \
                  in ("true", "1", "yes")
    rviz_config = LaunchConfiguration("rviz_config").perform(context)

    nodes = []

    # ── robot_state_publisher ─────────────────────────────────────────────
    # Publishes /robot_description  (consumed by RViz RobotModel display)
    # and broadcasts all fixed-joint TF transforms from the URDF.
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="kinect_model_publisher",
        output="screen",
        parameters=[{
            "robot_description": urdf,
            "use_sim_time":      False,
        }],
    ))

    # ── RViz ─────────────────────────────────────────────────────────────
    if do_rviz:
        rviz_args = ["-d", rviz_config] if os.path.isfile(rviz_config) else []
        if not rviz_args:
            print(
                f"[kinect_viz] RViz config not found at '{rviz_config}', "
                "launching with default layout."
            )
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
        ))

    return nodes


# ─────────────────────────────────────────────────────────────────────────── #
#  Entry point                                                                 #
# ─────────────────────────────────────────────────────────────────────────── #

def generate_launch_description():
    pkg_share = get_package_share_directory("kinect2_bridge")

    default_rviz = os.path.join(pkg_share, "launch", "kinect_viz.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description=(
                "Open RViz automatically. "
                "Set to 'false' when running as an overlay alongside "
                "dual_kinect2_simple.launch.py."
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description=(
                "Path to an RViz .rviz config file. "
                "Defaults to kinect_viz.rviz in the package launch directory. "
                "If the file does not exist RViz starts with its default layout."
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
