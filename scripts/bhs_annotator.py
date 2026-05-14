#!/usr/bin/env python3
"""
BHS detection overlay annotator.

Subscribes to /cinematic (raw camera from gz-sim), reads detection events
from /tmp/bag_events.jsonl, draws an Argus-style HUD on each frame, and
republishes to /cinematic_annotated for the recorder to capture.

HUD elements:
  - Top-left: scanner identifier + active sensors + BHS belt speed
  - Top-right: live clock + system status
  - Sensor frame outline (where bags are scanned)
  - Bottom-left: rolling "scan feed" of last N detections
  - Latest detection: zoomed callout when bag passes under sensor
"""
import json
import os
import time
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image


EVENTS_PATH = "/tmp/bag_events.jsonl"
MAX_FEED = 6  # how many past detections to show in side panel


class Annotator(Node):
    def __init__(self):
        super().__init__("bhs_annotator")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Image, "/cinematic", self.on_image, qos)
        self.pub = self.create_publisher(Image, "/cinematic_annotated", qos)
        self.events = []
        self.events_loaded_lines = 0
        self.start_wall = time.time()
        self.create_timer(0.5, self.reload_events)
        self.get_logger().info("annotator ready")

    def reload_events(self):
        if not os.path.exists(EVENTS_PATH):
            return
        try:
            with open(EVENTS_PATH) as f:
                lines = f.readlines()
        except OSError:
            return
        new_events = []
        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                new_events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
        self.events = new_events

    def overlay(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        now_wall = time.time()
        elapsed = now_wall - self.start_wall

        # ---- Top-left scanner header
        cv2.rectangle(frame, (20, 20), (520, 110), (15, 20, 30), -1)
        cv2.rectangle(frame, (20, 20), (520, 110), (90, 180, 255), 2)
        cv2.putText(frame, "ARGUS BHS SCANNER", (35, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "RGB stereo + Pol + Active IR | DB match",
                    (35, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180, 200, 230), 1, cv2.LINE_AA)
        cv2.putText(frame, "BHS speed: 25 m/min (0.417 m/s)",
                    (35, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 200, 230), 1, cv2.LINE_AA)

        # ---- Top-right status
        cv2.rectangle(frame, (w - 320, 20), (w - 20, 80), (15, 20, 30), -1)
        cv2.rectangle(frame, (w - 320, 20), (w - 20, 80), (60, 200, 90), 2)
        cv2.putText(frame, "STATUS: ONLINE",
                    (w - 305, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (60, 220, 90), 2, cv2.LINE_AA)
        cv2.putText(frame, f"t = {elapsed:6.2f} s",
                    (w - 305, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180, 230, 200), 1, cv2.LINE_AA)

        # ---- Sensor zone outline (where in image the sensor sees)
        # Approximate based on world-x ~= 2.0 maps to image around the sensor arch.
        # The cinematic camera is fixed at SDF pose (0.2 -2.8 1.55), so the sensor
        # arch is roughly at frame center horizontally, upper-middle vertically.
        sx, sy = int(w * 0.50), int(h * 0.45)
        sw, sh = int(w * 0.15), int(h * 0.22)
        cv2.rectangle(frame, (sx - sw // 2, sy - sh // 2),
                      (sx + sw // 2, sy + sh // 2), (0, 255, 200), 2)
        cv2.putText(frame, "SCAN ZONE", (sx - 65, sy - sh // 2 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 1, cv2.LINE_AA)

        # ---- Latest detection callout (large) — show event whose detect_t is
        # closest to now (within +/-2s window).
        latest = None
        for e in self.events:
            if abs((e["detect_t"]) - now_wall) < 1.6:
                latest = e
                break
        if latest is not None:
            self._draw_callout(frame, latest, w, h)

        # ---- Bottom-left rolling feed
        recent = [e for e in self.events if (e["detect_t"]) <= now_wall + 0.2]
        recent = sorted(recent, key=lambda e: e["detect_t"], reverse=True)[:MAX_FEED]
        self._draw_feed(frame, recent, w, h)
        return frame

    def _draw_callout(self, frame, e, w, h):
        # Glow box centered on scan zone
        sx, sy = int(w * 0.50), int(h * 0.45)
        # offset upward
        bx, by = sx + 120, sy - 60
        bw, bh = 540, 180
        overlay = frame.copy()
        cv2.rectangle(overlay, (bx, by), (bx + bw, by + bh), (10, 25, 40), -1)
        cv2.addWeighted(overlay, 0.82, frame, 0.18, 0, frame)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (60, 220, 255), 2)
        cv2.putText(frame, f"# {e['name']}",
                    (bx + 12, by + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (60, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"Class : {e['class']}    Material : {e['material']}",
                    (bx + 12, by + 62), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"Dims  : {e['L_mm']} x {e['W_mm']} x {e['H_mm']} mm",
                    (bx + 12, by + 92), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"DB match: {e['db_brand'][:34]}",
                    (bx + 12, by + 120), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 230, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"score = {e['db_score']:.3f}    mass {e['mass_kg']:.1f} kg",
                    (bx + 12, by + 148), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 230, 255), 1, cv2.LINE_AA)
        # connector line
        cv2.line(frame, (sx, sy), (bx, by + 40), (60, 220, 255), 1, cv2.LINE_AA)

    def _draw_feed(self, frame, recent, w, h):
        if not recent:
            return
        fx, fy = 30, h - 30 - 30 * (len(recent) + 1)
        fw = 720
        fh = 30 * (len(recent) + 1) + 12
        cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (12, 18, 26), -1)
        cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (90, 180, 255), 1)
        cv2.putText(frame, "DETECTION FEED  (Argus output -> BHS PLC / BSM)",
                    (fx + 10, fy + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (90, 180, 255), 1, cv2.LINE_AA)
        for i, e in enumerate(recent):
            line_y = fy + 22 + 28 * (i + 1)
            dims = f"{e['L_mm']}x{e['W_mm']}x{e['H_mm']}"
            text = f"{e['name'][:20]:20s} {e['class']:<2s}/{e['material']:<3s}  {dims:>15s} mm   {e['db_brand'][:18]:<18s} ({e['db_score']:.2f})"
            cv2.putText(frame, text, (fx + 10, line_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (220, 230, 240), 1, cv2.LINE_AA)

    def on_image(self, msg: Image):
        if msg.encoding not in ("rgb8", "RGB8", "bgr8", "BGR8"):
            return
        h, w = msg.height, msg.width
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((h, w, 3))
        if msg.encoding.lower() == "rgb8":
            bgr = arr[:, :, ::-1].copy()
        else:
            bgr = arr.copy()
        bgr = self.overlay(bgr)
        # publish back as rgb8 (matches what record_video.py expects)
        out = Image()
        out.header = msg.header
        out.height = h
        out.width = w
        out.encoding = "rgb8"
        out.is_bigendian = 0
        out.step = w * 3
        out.data = bgr[:, :, ::-1].tobytes()
        self.pub.publish(out)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = Annotator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
