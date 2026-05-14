#!/usr/bin/env python3
"""
Subscribe to a ROS sensor_msgs/Image topic and pipe raw frames to ffmpeg,
encoding to H.264 mp4 at the topic's native resolution.

Usage:
  record_video.py <topic> <output.mp4> <duration_s> [width=1920] [height=1080]
"""
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, topic, out_path, width, height, duration_s):
        super().__init__("video_recorder")
        self.out_path = out_path
        self.duration_s = duration_s
        self.width = width
        self.height = height
        self.t0 = None
        self.frames_in = 0
        self.ff = None
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Image, topic, self.on_image, qos)
        self.get_logger().info(f"subscribed to {topic} (BEST_EFFORT)")

    def _start_ffmpeg(self, w, h):
        # Use fragmented MP4 so each fragment is independently decodable;
        # the file remains playable even if the recorder is killed before EOS.
        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo",
            "-pixel_format", "rgb24",
            "-video_size", f"{w}x{h}",
            "-framerate", "30",
            "-i", "-",
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            "-preset", "veryfast",
            "-crf", "20",
            "-g", "30",  # keyframe per second
            "-movflags", "+frag_keyframe+empty_moov+default_base_moof",
            self.out_path,
        ]
        self.get_logger().info("ffmpeg: " + " ".join(cmd))
        self.ff = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def on_image(self, msg: Image):
        if self.t0 is None:
            self.t0 = time.time()
            self.width = msg.width
            self.height = msg.height
            self._start_ffmpeg(self.width, self.height)
            self.get_logger().info(
                f"first frame: {msg.width}x{msg.height} encoding={msg.encoding}")
        data = bytes(msg.data)
        if msg.encoding in ("rgb8", "RGB8"):
            pass
        elif msg.encoding in ("bgr8", "BGR8"):
            arr = np.frombuffer(data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            data = arr[:, :, ::-1].tobytes()
        try:
            self.ff.stdin.write(data)
            self.frames_in += 1
        except BrokenPipeError:
            self.get_logger().error("ffmpeg pipe broken")
            self._close()
            raise SystemExit(1)
        if (time.time() - self.t0) >= self.duration_s:
            self.get_logger().info(f"duration reached, frames={self.frames_in}")
            self._close()
            rclpy.shutdown()

    def _close(self):
        if self.ff is None:
            return
        try:
            self.ff.stdin.close()
        except Exception:
            pass
        try:
            self.ff.wait(timeout=30)
        except subprocess.TimeoutExpired:
            self.ff.kill()
        self.ff = None
        self.get_logger().info(f"wrote {self.out_path}")


def main():
    if len(sys.argv) < 4:
        print("usage: record_video.py <topic> <out.mp4> <duration_s> [width] [height]")
        sys.exit(2)
    topic = sys.argv[1]
    out = sys.argv[2]
    dur = float(sys.argv[3])
    w = int(sys.argv[4]) if len(sys.argv) > 4 else 1920
    h = int(sys.argv[5]) if len(sys.argv) > 5 else 1080
    rclpy.init()
    node = Recorder(topic, out, w, h, dur)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node._close()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
