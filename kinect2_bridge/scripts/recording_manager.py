#!/usr/bin/env python3
"""Service-driven recording manager.

Exposes start_recording / stop_recording services that launch the colour video
recorder and rosbag record processes for a configured set of topics.
"""

import os
import signal
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RecordingManager(Node):
    def __init__(self):
        super().__init__("recording_manager")

        self.declare_parameter("output_root", os.path.expanduser("~/data"))
        self.declare_parameter("participant_id", "0000")
        self.declare_parameter("color_topics", [""])
        self.declare_parameter("bag_topics", [""])
        self.declare_parameter("video_encoder", "hevc_nvenc")
        self.declare_parameter("video_crf", 18)
        self.declare_parameter("video_fps", 30.0)
        self.declare_parameter("bag_storage", "mcap")
        self.declare_parameter("bag_compression", "none")
        self.declare_parameter("bag_cache_size_mb", 128)

        self._bag_cache_size_mb = int(self.get_parameter("bag_cache_size_mb").value)
        self._output_root = str(self.get_parameter("output_root").value)
        self._participant_id = str(self.get_parameter("participant_id").value)
        self._color_topics = [
            str(t) for t in self.get_parameter("color_topics").value if t
        ]
        self._bag_topics = list(
            dict.fromkeys(str(t) for t in self.get_parameter("bag_topics").value if t)
        )
        self._video_encoder = str(self.get_parameter("video_encoder").value)
        self._video_crf = int(self.get_parameter("video_crf").value)
        self._video_fps = float(self.get_parameter("video_fps").value)
        self._bag_storage = str(self.get_parameter("bag_storage").value)
        self._bag_compression = str(self.get_parameter("bag_compression").value).lower()
        if self._bag_compression not in {"none", "file", "message"}:
            self.get_logger().warn(
                f"bag_compression='{self._bag_compression}' not recognized; "
                "treating as 'none'. Valid values: none, file, message."
            )
            self._bag_compression = "none"

        self._lock = threading.Lock()
        self._active = False
        self._stop_requested = False
        self._session_dir = ""
        self._color_proc = None
        self._bag_proc = None

        self._start_service = self.create_service(
            Trigger, "start_recording", self._on_start
        )
        self._stop_service = self.create_service(
            Trigger, "stop_recording", self._on_stop
        )

        self.get_logger().info(
            "Recording manager ready\n"
            f"  color_topics: {self._color_topics}\n"
            f"  bag_topics   : {self._bag_topics}\n"
            f"  output_root  : {self._output_root}\n"
            f"  participant  : {self._participant_id}\n"
            f"  encoder      : {self._video_encoder}\n"
            f"  crf          : {self._video_crf}\n"
            f"  fps          : {self._video_fps}\n"
            f"  bag_storage  : {self._bag_storage}\n"
            f"  bag_compression: {self._bag_compression}\n"
            f"  cache_size_mb: {self._bag_cache_size_mb}"
        )

    def _session_path(self) -> str:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self._output_root, self._participant_id, f"session_{stamp}")

    def _start_process(self, cmd: list[str]) -> subprocess.Popen:
        return subprocess.Popen(
            cmd,
            stdout=None,
            stderr=None,
            preexec_fn=os.setsid,
        )

    def _build_color_cmd(self, output_dir: str) -> list[str]:
        topic_arg = "[" + ",".join(self._color_topics) + "]"
        return [
            "ros2",
            "run",
            "kinect2_bridge",
            "colour_video_recorder.py",
            "--ros-args",
            "-p",
            f"topics:={topic_arg}",
            "-p",
            f"output_dir:={output_dir}",
            "-p",
            f"encoder:={self._video_encoder}",
            "-p",
            f"crf:={self._video_crf}",
            "-p",
            f"fps:={self._video_fps}",
        ]

    def _build_bag_cmd(self, bag_dir: str) -> list[str]:
        cmd = [
            "ros2",
            "bag",
            "record",
            "-o",
            bag_dir,
            "--storage",
            self._bag_storage,
            "--max-cache-size",
            str(self._bag_cache_size_mb * 1024 * 1024),
        ]
        if self._bag_compression in {"file", "message"}:
            cmd += [
                "--compression-mode", self._bag_compression,
                "--compression-format", "zstd",
            ]
        cmd += ["--topics", *self._bag_topics]
        return cmd

    def _stop_processes(self):
        with self._lock:
            color_proc = self._color_proc
            bag_proc = self._bag_proc
            session_dir = self._session_dir
            self._active = False
            self._stop_requested = True
            self._color_proc = None
            self._bag_proc = None
            self._session_dir = ""

        for proc in (color_proc, bag_proc):
            if proc is None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass

        deadline = time.time() + 30.0
        for proc in (color_proc, bag_proc):
            if proc is None:
                continue
            remaining = max(0.0, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass

        return session_dir

    def _on_start(self, request, response):
        del request
        with self._lock:
            if self._active:
                response.success = False
                response.message = "Recording already active"
                return response
            if not self._color_topics:
                response.success = False
                response.message = "No color topics configured"
                return response
            if not self._bag_topics:
                response.success = False
                response.message = "No bag topics configured"
                return response

            self._session_dir = self._session_path()
            output_dir = self._session_dir
            bag_dir = os.path.join(output_dir, "depth")
            # ros2 bag record refuses to start if its output folder already
            # exists, so only create the session dir and let it make bag_dir.
            os.makedirs(output_dir, exist_ok=True)
            self._stop_requested = False

        try:
            self._color_proc = self._start_process(self._build_color_cmd(output_dir))
            self._bag_proc = self._start_process(self._build_bag_cmd(bag_dir))
        except Exception as exc:
            self.get_logger().error(f"Failed to start recording: {exc}")
            self._stop_processes()
            response.success = False
            response.message = f"Failed to start recording: {exc}"
            return response

        with self._lock:
            self._active = True

        self.get_logger().info(f"Recording started in {output_dir}")
        response.success = True
        response.message = f"Recording started in {output_dir}"
        return response

    def _on_stop(self, request, response):
        del request
        with self._lock:
            if not self._active:
                response.success = False
                response.message = "No active recording"
                return response

        session_dir = self._stop_processes()
        self.get_logger().info(f"Recording stopped: {session_dir}")
        response.success = True
        response.message = f"Recording stopped: {session_dir}"
        return response

    def destroy_node(self):
        if self._active:
            self._stop_processes()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecordingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
