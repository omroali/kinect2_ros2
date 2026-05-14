#!/usr/bin/env python3
"""
H.265 Video → ROS Image Publisher
====================================
Replays one or more H.265 colour videos recorded by colour_video_recorder.py
back into ROS as sensor_msgs/Image topics, with timestamps restored from the
companion CSV files.

This is the missing piece between the H.265 recordings and the pointcloud
reconstruction pipeline (pointcloud_from_bag.launch.py).  When run alongside
a depth bag replay, depth_image_proc can reconstruct PointCloud2 exactly as
if the colour frames had been recorded in the bag directly.

How timestamp sync works
------------------------
The bag is replayed with --clock, which drives /clock forward at the bag's
original time.  This node subscribes to /clock and publishes each colour frame
at the moment /clock reaches that frame's original ros_timestamp_ns from the
CSV.  This means depth_image_proc's approximate time synchroniser sees colour
and depth frames with matching header stamps and pairs them correctly —
regardless of how fast or slow the bag is playing back.

Usage
-----
# Single camera — run alongside pointcloud_from_bag.launch.py
ros2 run kinect2_bridge video_to_image_publisher.py \
    --ros-args \
    -p video:="/data/session_001/kinect2_1_qhd_image_color_rect.mp4" \
    -p timestamps:="/data/session_001/kinect2_1_qhd_image_color_rect.csv" \
    -p topic:="/kinect2_1/qhd/image_color_rect" \
    -p use_sim_time:=true

# Four cameras at once
ros2 run kinect2_bridge video_to_image_publisher.py \
    --ros-args \
    -p videos:="[/data/s1/kinect2_1_qhd_image_color_rect.mp4, \
                  /data/s1/kinect2_2_qhd_image_color_rect.mp4, \
                  /data/s1/kinect2_3_qhd_image_color_rect.mp4, \
                  /data/s1/kinect2_4_qhd_image_color_rect.mp4]" \
    -p timestamp_csvs:="[/data/s1/kinect2_1_qhd_image_color_rect.csv, \
                          /data/s1/kinect2_2_qhd_image_color_rect.csv, \
                          /data/s1/kinect2_3_qhd_image_color_rect.csv, \
                          /data/s1/kinect2_4_qhd_image_color_rect.csv]" \
    -p topics:="[/kinect2_1/qhd/image_color_rect, \
                  /kinect2_2/qhd/image_color_rect, \
                  /kinect2_3/qhd/image_color_rect, \
                  /kinect2_4/qhd/image_color_rect]" \
    -p frame_ids:="[kinect2_1_rgb_optical_frame, \
                    kinect2_2_rgb_optical_frame, \
                    kinect2_3_rgb_optical_frame, \
                    kinect2_4_rgb_optical_frame]" \
    -p use_sim_time:=true


ros2 run kinect2_bridge video_to_image_publisher.py \
  --ros-args \
  -p videos:="[/home/ros/base_ws/src/kinect2_ros2/kinect2_3_qhd_image_color_rect.mp4]" \
  -p timestamp_csvs:="[/home/ros/base_ws/src/kinect2_ros2/kinect2_3_qhd_image_color_rect.csv]" \
  -p topics:="[/kinect2_3/qhd/image_color_rect]" \
  -p frame_ids:="[kinect2_3_rgb_optical_frame]" \
  -p use_sim_time:=false


Parameters
----------
Single-stream mode (convenience):
  video            str        Path to .mp4 file
  timestamps       str        Path to companion .csv file
  topic            str        ROS topic to publish on

Multi-stream mode:
  videos           list[str]  Paths to .mp4 files  (must match length of topics)
  timestamp_csvs   list[str]  Paths to .csv files  (must match length of topics)
  topics           list[str]  ROS topics to publish on
  frame_ids        list[str]  Header frame_id for each topic (optional)

  queue_size       int        Publisher queue depth. Default: 10
  encoding         str        ROS image encoding.   Default: bgr8
                              Use rgb8 for non-Kinect cameras.

Full reconstruction pipeline
-----------------------------
Terminal 1 — depth bag:
  ros2 bag play /data/session_001/depth --clock \
      --topics /kinect2_1/qhd/image_depth_rect/compressed \
               /kinect2_1/qhd/camera_info

Terminal 2 — colour video republisher (this node):
  ros2 run kinect2_bridge video_to_image_publisher.py --ros-args \
      -p video:="/data/session_001/kinect2_1_qhd_image_color_rect.mp4" \
      -p timestamps:="/data/session_001/kinect2_1_qhd_image_color_rect.csv" \
      -p topic:="/kinect2_1/qhd/image_color_rect" \
      -p use_sim_time:=true

Terminal 3 — depth decompressor (if bag has compressed depth):
  ros2 run image_transport republish compressed raw \
      --ros-args \
      --remap in/compressed:=/kinect2_1/qhd/image_depth_rect/compressed \
      --remap out:=/kinect2_1/qhd/image_depth_rect \
      -p use_sim_time:=true

Terminal 4 — pointcloud reconstruction:
  ros2 launch kinect2_bridge pointcloud_from_bag.launch.py \
      bag:=/dev/null \
      namespace:=kinect2_1 \
      launch_rviz:=true
  # Note: pass bag:=/dev/null since the bag is already playing in Terminal 1
"""

import csv
import os
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


# ── Timestamp CSV loader ──────────────────────────────────────────────────── #

def load_timestamps(csv_path: str) -> list[tuple[int, int]]:
    """
    Load per-frame timestamps from a colour_video_recorder.py CSV.
    Returns list of (frame_idx, ros_timestamp_ns) sorted by frame_idx.
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Timestamp CSV not found: {csv_path}")

    rows = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((int(row['frame_idx']), int(row['ros_timestamp_ns'])))

    rows.sort(key=lambda r: r[0])
    return rows


def make_default_camera_info(width: int, height: int) -> CameraInfo:
    info = CameraInfo()
    info.width = width
    info.height = height
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

    fx = float(max(width, height))
    fy = fx
    cx = float(width - 1) / 2.0
    cy = float(height - 1) / 2.0

    info.k = [
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0,
    ]
    info.r = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    info.p = [
        fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return info


# ── Per-stream player ─────────────────────────────────────────────────────── #

class VideoStream:
    """
    Manages playback of one H.265 video file.

    Frames are held in a cursor — we don't decode everything upfront.
    cv2.VideoCapture with CAP_PROP_POS_FRAMES lets us seek cheaply for H.265
    because most frames in a static-background scene are P-frames; seeking to
    the nearest keyframe and stepping forward is fast in practice.

    The stream exposes next_ts_ns so the node can cheaply check whether the
    current sim clock has reached this stream's next frame without decoding it.
    """

    def __init__(self, video_path: str, csv_path: str,
                 topic: str, encoding: str, queue_size: int, frame_id: str, node: Node):
        self.topic    = topic
        self.encoding = encoding
        self.frame_id = frame_id
        self._node    = node

        self._timestamps = load_timestamps(csv_path)
        self._cursor     = 0                        # index into _timestamps
        self._total      = len(self._timestamps)

        self._cap = cv2.VideoCapture(video_path)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open video: {video_path}")

        self._pub = node.create_publisher(Image, topic, queue_size)
        self._info_pub = node.create_publisher(CameraInfo, self._camera_info_topic(topic), queue_size)
        self._camera_info = make_default_camera_info(
            int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 0,
            int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 0,
        )
        self._lock = threading.Lock()

        node.get_logger().info(
            f"VideoStream ready: {os.path.basename(video_path)}\n"
            f"  → {topic}  ({self._total} frames)"
        )

    @property
    def finished(self) -> bool:
        return self._cursor >= self._total

    @property
    def next_ts_ns(self) -> int | None:
        """ROS timestamp of the next frame to publish, or None if finished."""
        if self.finished:
            return None
        return self._timestamps[self._cursor][1]

    def publish_if_due(self, clock_ns: int) -> bool:
        """
        Decode and publish the next frame if sim clock has reached its
        timestamp.  Returns True if a frame was published.
        """
        with self._lock:
            if self.finished:
                return False

            frame_idx, ts_ns = self._timestamps[self._cursor]

            if clock_ns < ts_ns:
                return False   # not yet time for this frame

            # Seek to correct frame position.
            # For sequential playback this is usually a no-op since OpenCV
            # tracks position internally, but explicit seek handles rate
            # changes and any dropped frames correctly.
            current_pos = int(self._cap.get(cv2.CAP_PROP_POS_FRAMES))
            if current_pos != frame_idx:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)

            ok, frame = self._cap.read()
            if not ok:
                self._node.get_logger().warn(
                    f"[{self.topic}] Failed to read frame {frame_idx} — skipping."
                )
                self._cursor += 1
                return False

            # Build ROS Image message with original timestamp restored
            msg = Image()
            msg.header = Header()
            msg.header.stamp.sec     = ts_ns // 1_000_000_000
            msg.header.stamp.nanosec = ts_ns  % 1_000_000_000
            msg.header.frame_id      = self.frame_id
            msg.height   = frame.shape[0]
            msg.width    = frame.shape[1]
            msg.encoding = self.encoding
            msg.step     = frame.shape[1] * frame.shape[2]
            msg.data     = frame.tobytes()

            self._pub.publish(msg)
            info = self._camera_info
            info.header = Header()
            info.header.stamp.sec = ts_ns // 1_000_000_000
            info.header.stamp.nanosec = ts_ns % 1_000_000_000
            info.header.frame_id = self.frame_id
            self._info_pub.publish(info)
            self._cursor += 1
            return True

    def destroy(self):
        self._cap.release()

    @staticmethod
    def _camera_info_topic(image_topic: str) -> str:
        if image_topic.endswith("/image_color_rect"):
            return image_topic[: -len("/image_color_rect")] + "/camera_info"
        return image_topic.rstrip("/") + "/camera_info"


# ── ROS2 Node ─────────────────────────────────────────────────────────────── #

class VideoToImagePublisher(Node):
    """
    Subscribes to /clock (published by the bag player) and publishes colour
    frames from H.265 videos at their original ROS timestamps.

    One VideoStream per camera runs inside a single node.  A tight timer
    loop checks all streams on every tick — this is cheap because VideoStream
    only decodes a frame when the clock has actually reached its timestamp.
    """

    def __init__(self):
        super().__init__('video_to_image_publisher')

        # ── Parameters ───────────────────────────────────────────────────── #

        # Single-stream convenience parameters
        self.declare_parameter('video',      '')
        self.declare_parameter('timestamps', '')
        self.declare_parameter('topic',      '')

        # Multi-stream parameters
        self.declare_parameter('videos',         [''])
        self.declare_parameter('timestamp_csvs', [''])
        self.declare_parameter('topics',         [''])
        self.declare_parameter('frame_id',       '')
        self.declare_parameter('frame_ids',      [''])

        self.declare_parameter('queue_size', 10)
        self.declare_parameter('encoding',   'bgr8')

        queue_size = self.get_parameter('queue_size').value
        encoding   = self.get_parameter('encoding').value
        single_frame_id = self.get_parameter('frame_id').value

        # ── Resolve single vs multi-stream ────────────────────────────────── #

        single_video = self.get_parameter('video').value
        single_csv   = self.get_parameter('timestamps').value
        single_topic = self.get_parameter('topic').value

        if single_video and single_csv and single_topic:
            # Single-stream mode
            videos  = [single_video]
            csvs    = [single_csv]
            topics  = [single_topic]
        else:
            videos  = self.get_parameter('videos').value
            csvs    = self.get_parameter('timestamp_csvs').value
            topics  = self.get_parameter('topics').value
            frame_ids = self.get_parameter('frame_ids').value

            if len(frame_ids) == 1 and len(topics) > 1:
                frame_ids = frame_ids * len(topics)
            elif not frame_ids:
                frame_ids = [''] * len(topics)

            if not (len(videos) == len(csvs) == len(topics) == len(frame_ids)):
                raise ValueError(
                    f"videos ({len(videos)}), timestamp_csvs ({len(csvs)}), "
                    f"topics ({len(topics)}), and frame_ids ({len(frame_ids)}) must all have the same length."
                )
            # Filter out empty default values
            videos = [v for v in videos if v]
            csvs   = [c for c in csvs   if c]
            topics = [t for t in topics if t]

        if not videos:
            raise ValueError(
                "No video files specified.  Provide either:\n"
                "  -p video:=... -p timestamps:=... -p topic:=...\n"
                "or\n"
                "  -p videos:=[...] -p timestamp_csvs:=[...] -p topics:=[...] -p frame_ids:=[...]"
            )

        # ── Create one VideoStream per camera ─────────────────────────────── #

        if single_video and single_csv and single_topic:
            frame_ids = [single_frame_id]

        self._streams: list[VideoStream] = []
        for video, csv_path, topic, frame_id in zip(videos, csvs, topics, frame_ids):
            stream = VideoStream(
                video_path = video,
                csv_path   = csv_path,
                topic      = topic,
                encoding   = encoding,
                queue_size = queue_size,
                frame_id   = frame_id,
                node       = self,
            )
            self._streams.append(stream)

        self._use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self._clock_ns: int = 0
        self._clock_lock    = threading.Lock()
        self._wall_start_mono_ns: int | None = None
        self._wall_start_ros_ns: int | None = None

        if self._use_sim_time:
            # /clock is published by 'ros2 bag play --clock'.
            self.create_subscription(Clock, '/clock', self._clock_callback, 10)
        else:
            first_ts = [s.next_ts_ns for s in self._streams if s.next_ts_ns is not None]
            self._wall_start_ros_ns = min(first_ts) if first_ts else 0
            self._wall_start_mono_ns = time.monotonic_ns()

        # ── Publish timer ─────────────────────────────────────────────────── #
        # Poll at 1ms intervals — fast enough to not miss frames at 30fps
        # (33ms per frame), cheap because VideoStream.publish_if_due is O(1)
        # when no frame is due.

        self._timer = self.create_timer(0.001, self._tick)

        if self._use_sim_time:
            self.get_logger().info(
                f"Video publisher ready — {len(self._streams)} stream(s)\n"
                "Waiting for /clock from bag player..."
            )
        else:
            self.get_logger().info(
                f"Video publisher ready — {len(self._streams)} stream(s)\n"
                "Wall-time mode active (use_sim_time=false)."
            )

    def _clock_callback(self, msg: Clock):
        ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        with self._clock_lock:
            self._clock_ns = ns

    def _tick(self):
        if self._use_sim_time:
            with self._clock_lock:
                clock_ns = self._clock_ns
            if clock_ns == 0:
                return  # bag not started yet
        else:
            if self._wall_start_mono_ns is None or self._wall_start_ros_ns is None:
                return
            elapsed_ns = time.monotonic_ns() - self._wall_start_mono_ns
            clock_ns = self._wall_start_ros_ns + elapsed_ns

        all_finished = True
        for stream in self._streams:
            stream.publish_if_due(clock_ns)
            if not stream.finished:
                all_finished = False

        if all_finished:
            self.get_logger().info("All video streams finished.")
            self._timer.cancel()

    def destroy_node(self):
        for stream in self._streams:
            stream.destroy()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = VideoToImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
