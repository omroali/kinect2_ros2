#!/usr/bin/env python3
"""
H.265 Colour Video Recorder
============================
Subscribes to one or more RGB image topics and encodes each stream to a
separate H.265 (.mp4) file via an ffmpeg subprocess pipe.  A companion
CSV timestamp file is written alongside each video so that every frame
can be matched back to its original ROS timestamp during ML inference.

Works with any sensor that publishes sensor_msgs/Image in BGR8 or RGB8
encoding — Kinect v2 (kinect2_bridge), RealSense, USB cameras, etc.

Usage
-----
# Kinect v2 — single sensor, default topic
ros2 run kinect2_bridge colour_video_recorder.py

# Kinect v2 — explicit topic and output path
ros2 run kinect2_bridge colour_video_recorder.py \
    --ros-args \
    -p topics:="[/kinect2/qhd/image_color_rect]" \
    -p output_dir:="/data/recordings" \
    -p crf:=28

# Four Kinects at once
ros2 run kinect2_bridge colour_video_recorder.py \
    --ros-args \
    -p topics:="[/kinect2_1/qhd/image_color_rect, \
                  /kinect2_2/qhd/image_color_rect, \
                  /kinect2_3/qhd/image_color_rect, \
                  /kinect2_4/qhd/image_color_rect]" \
    -p output_dir:="/data/recordings"

ros2 run kinect2_bridge colour_video_recorder.py \
    --ros-args \
    -p topics:="[/kinect2_3/qhd/image_color_rect]" \
    -p output_dir:="/data/recordings"

# Generic RGB camera (publishes RGB8 instead of BGR8)
ros2 run kinect2_bridge colour_video_recorder.py \
    --ros-args \
    -p topics:="[/camera/color/image_raw]"

Parameters
----------
topics      list[str]   ROS image topics to record.
                        Default: [/kinect2/qhd/image_color_rect]
output_dir  str         Directory to write .mp4 and .csv files.
                        Default: ~/kinect2_recordings
crf         int         H.265 constant rate factor. Lower = better quality,
                        larger file.  18=visually lossless, 28=good, 35=small.
                        Default: 28
encoder     str         ffmpeg encoder to use.
                        CPU:    libx265  (always available, slow)
                        Nvidia: hevc_nvenc
                        Intel:  hevc_qsv
                        AMD:    hevc_amf
                        Default: libx265
fps         float       Frames per second written to the video container.
                        Should match the sensor's actual publish rate.
                        Default: 30.0
queue_size  int         ROS subscriber queue depth.  Default: 5

Output files (per topic)
------------------------
  <output_dir>/<safe_topic_name>.mp4   H.265 encoded colour video
  <output_dir>/<safe_topic_name>.csv   Per-frame ROS timestamps

  The CSV has two columns:
    frame_idx        0-based frame counter
    ros_timestamp_ns ROS header stamp in nanoseconds

  Use ros_timestamp_ns to align a video frame with a depth frame from
  the companion bag recording (see pointcloud_from_bag.launch.py).
"""

import os
import csv
import subprocess
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


# ── Helpers ──────────────────────────────────────────────────────────────── #

def topic_to_filename(topic: str) -> str:
    """Convert a ROS topic path to a safe filename stem.
    e.g. /kinect2_1/qhd/image_color_rect → kinect2_1_qhd_image_color_rect
    """
    return topic.strip('/').replace('/', '_')


# ── Per-stream recorder ───────────────────────────────────────────────────── #

class StreamRecorder:
    """
    Manages one ffmpeg process + one CSV file for a single image topic.

    Adapting to a different sensor
    ───────────────────────────────
    The only thing sensor-specific here is the pixel format passed to ffmpeg:
      - kinect2_bridge publishes BGR8  → pixel_fmt = 'bgr24'
      - Most other cameras (RealSense, USB) publish RGB8 → pixel_fmt = 'rgb24'
    This is detected automatically from the first message encoding field.
    If your camera uses a different encoding (e.g. yuv422), add a branch in
    _pixel_fmt_from_encoding() below.
    """

    def __init__(self, topic: str, output_dir: str, crf: int,
                 encoder: str, fps: float, logger):
        self.topic      = topic
        self.logger     = logger
        self.frame_idx  = 0
        self._proc      = None
        self._csv_file  = None
        self._csv_writer = None
        self._lock      = threading.Lock()
        self._started   = False

        stem = topic_to_filename(topic)
        self._video_path = os.path.join(output_dir, f"{stem}.mp4")
        self._csv_path   = os.path.join(output_dir, f"{stem}.csv")
        self._crf        = crf
        self._encoder    = encoder
        self._fps        = fps

    def _pixel_fmt_from_encoding(self, encoding: str) -> str:
        """
        Map a ROS image encoding string to an ffmpeg pixel format name.

        Extend this if you add sensors with other encodings:
          'bgra8'   → 'bgra'      (some depth cameras, 4-channel)
          'mono8'   → 'gray'      (greyscale cameras)
          'yuv422'  → 'uyvy422'   (some industrial cameras)
        """
        mapping = {
            'bgr8':  'bgr24',   # kinect2_bridge default
            'rgb8':  'rgb24',   # RealSense, most USB cameras
            'bgra8': 'bgra',
            'rgba8': 'rgba',
            'mono8': 'gray',
        }
        fmt = mapping.get(encoding.lower())
        if fmt is None:
            self.logger.warn(
                f"[{self.topic}] Unknown encoding '{encoding}', assuming bgr24. "
                "Add it to _pixel_fmt_from_encoding() if output looks wrong."
            )
            fmt = 'bgr24'
        return fmt

    def _start(self, width: int, height: int, pixel_fmt: str):
        """Lazily start the ffmpeg process on the first received frame."""

        # ── CSV timestamp file ────────────────────────────────────────────── #
        self._csv_file   = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['frame_idx', 'ros_timestamp_ns'])

        # ── ffmpeg command ────────────────────────────────────────────────── #
        # stdin  ← raw pixel data piped frame-by-frame from this node
        # stdout → suppressed
        # stderr → suppressed (change to None to see ffmpeg logs)
        #
        # -vf format=yuv420p  ensures broad player/decoder compatibility.
        # For Nvidia hevc_nvenc replace -crf with -cq (same scale).
        #
        # Hardware encoder notes:
        #   hevc_nvenc:  replace '-crf N' with '-cq N'
        #   hevc_qsv:    replace '-crf N' with '-global_quality N'
        #   hevc_amf:    replace '-crf N' with '-qp_i N -qp_p N'

        if self._encoder == 'hevc_nvenc':
            quality_flags = ['-cq', str(self._crf)]
        elif self._encoder == 'hevc_qsv':
            quality_flags = ['-global_quality', str(self._crf)]
        elif self._encoder == 'hevc_amf':
            quality_flags = ['-qp_i', str(self._crf), '-qp_p', str(self._crf)]
        else:
            quality_flags = ['-crf', str(self._crf)]

        cmd = [
            'ffmpeg', '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', pixel_fmt,
            '-s', f'{width}x{height}',
            '-r', str(self._fps),
            '-i', 'pipe:0',
            '-vcodec', self._encoder,
            '-vf', 'format=yuv420p',
            *quality_flags,
            '-movflags', '+faststart',   # index at front for streaming/seeking
            self._video_path,
        ]

        self.logger.info(
            f"[{self.topic}] Starting ffmpeg → {self._video_path}\n"
            f"  encoder={self._encoder}  crf={self._crf}  "
            f"  {width}x{height} @ {self._fps}fps  pixel_fmt={pixel_fmt}"
        )

        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._started = True

    def write(self, msg: Image):
        """Write one frame to ffmpeg stdin and log its timestamp."""
        with self._lock:
            if not self._started:
                pixel_fmt = self._pixel_fmt_from_encoding(msg.encoding)
                self._start(msg.width, msg.height, pixel_fmt)

            # Raw pixel bytes → ffmpeg stdin
            try:
                self._proc.stdin.write(bytes(msg.data))
            except BrokenPipeError:
                self.logger.error(
                    f"[{self.topic}] ffmpeg pipe closed unexpectedly. "
                    "Check ffmpeg is installed and the encoder is available."
                )
                return

            # ROS timestamp in nanoseconds → CSV
            ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self._csv_writer.writerow([self.frame_idx, ts_ns])
            self.frame_idx += 1

    def stop(self):
        """Flush and close ffmpeg + CSV cleanly."""
        with self._lock:
            if not self._started:
                return
            if rclpy.ok():
                self.logger.info(
                    f"[{self.topic}] Stopping — {self.frame_idx} frames written."
                )
            try:
                self._proc.stdin.close()
                self._proc.wait(timeout=30)
            except Exception as e:
                if rclpy.ok():
                    self.logger.warn(f"[{self.topic}] ffmpeg shutdown error: {e}")
                self._proc.kill()
            if self._csv_file:
                self._csv_file.close()
            if rclpy.ok():
                self.logger.info(
                    f"[{self.topic}] Saved:\n"
                    f"  video: {self._video_path}\n"
                    f"  timestamps: {self._csv_path}"
                )


# ── ROS2 Node ─────────────────────────────────────────────────────────────── #

class ColourVideoRecorder(Node):

    def __init__(self):
        super().__init__('colour_video_recorder')

        # ── Parameters ───────────────────────────────────────────────────── #
        self.declare_parameter(
            'topics',
            ['/kinect2/qhd/image_color_rect'],
        )
        self.declare_parameter(
            'output_dir',
            os.path.expanduser('~/kinect2_recordings'),
        )
        self.declare_parameter('crf',      28)
        self.declare_parameter('encoder', 'libx265')
        self.declare_parameter('fps',      30.0)
        self.declare_parameter('queue_size', 5)

        topics     = self.get_parameter('topics').value
        output_dir = self.get_parameter('output_dir').value
        crf        = self.get_parameter('crf').value
        encoder    = self.get_parameter('encoder').value
        fps        = self.get_parameter('fps').value
        queue_size = self.get_parameter('queue_size').value

        os.makedirs(output_dir, exist_ok=True)

        # ── One StreamRecorder + one subscriber per topic ─────────────────── #
        self._recorders = {}

        for topic in topics:
            recorder = StreamRecorder(
                topic      = topic,
                output_dir = output_dir,
                crf        = crf,
                encoder    = encoder,
                fps        = fps,
                logger     = self.get_logger(),
            )
            self._recorders[topic] = recorder

            # Closure to capture the correct recorder per topic
            def make_callback(rec):
                def callback(msg: Image):
                    rec.write(msg)
                return callback

            self.create_subscription(
                Image,
                topic,
                make_callback(recorder),
                queue_size,
            )
            self.get_logger().info(f"Subscribed to {topic}")

        self.get_logger().info(
            f"Colour video recorder ready — {len(topics)} stream(s)\n"
            f"  output_dir : {output_dir}\n"
            f"  encoder    : {encoder}\n"
            f"  crf        : {crf}\n"
            f"  fps        : {fps}"
        )

    def destroy_node(self):
        for recorder in self._recorders.values():
            recorder.stop()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = ColourVideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
