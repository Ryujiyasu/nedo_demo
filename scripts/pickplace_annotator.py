#!/usr/bin/env python3
"""
Contest 3 Pick & Place HUD annotator.

Subscribes to /cinematic, reads phase events from /tmp/arm_events.jsonl,
draws an "Argus Stacking Robot" HUD, and republishes to /cinematic_annotated.
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
from sensor_msgs.msg import Image


EVENTS_PATH = "/tmp/arm_events.jsonl"
PHASE_LABELS = {
    "init": "INITIALIZING",
    "home_initial": "HOME (init)",
    "pregrasp": "PRE-GRASP   (approach over BHS conveyor)",
    "grasp": "GRASP       (descend onto bag)",
    "attach": "PICK-UP     (gripper close, bag attached)",
    "attach_fail": "PICK-UP    FAIL",
    "lift": "LIFT        (clear of conveyor)",
    "preplace": "TRANSIT     (rotate toward ULD)",
    "place": "PLACE       (descend into ULD)",
    "detach": "RELEASE     (open gripper, place bag)",
}


class Annotator(Node):
    def __init__(self):
        super().__init__("pickplace_annotator")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(Image, "/cinematic", self.on_image, qos)
        self.pub = self.create_publisher(Image, "/cinematic_annotated", qos)
        self.events = []
        self.start_wall = time.time()
        self.create_timer(0.3, self.reload_events)
        self.placed = []
        self.get_logger().info("pickplace annotator ready")

    def reload_events(self):
        if not os.path.exists(EVENTS_PATH):
            return
        try:
            with open(EVENTS_PATH) as f:
                lines = f.readlines()
        except OSError:
            return
        evs = []
        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                evs.append(json.loads(line))
            except json.JSONDecodeError:
                continue
        self.events = evs

    def current(self, now_wall):
        """Return the latest event <= now."""
        cur = None
        for e in self.events:
            if e["t"] <= now_wall + 0.05:
                cur = e
            else:
                break
        return cur

    def bag_info(self, name):
        for e in self.events:
            if e.get("phase") == "cycle_start" and e.get("bag_name") == name:
                return e
        return None

    def placed_so_far(self, now_wall):
        out = []
        for e in self.events:
            if e["t"] > now_wall:
                continue
            if e.get("phase") == "detach":
                info = self.bag_info(e["bag_name"])
                if info:
                    out.append(info)
        return out

    def overlay(self, frame):
        h, w = frame.shape[:2]
        now_wall = time.time()
        elapsed = now_wall - self.start_wall
        cur = self.current(now_wall)
        total_cycles = 0
        for e in self.events:
            if e.get("phase") == "init":
                total_cycles = e.get("total_cycles", 0)
                break

        # ---- Top-left: header
        cv2.rectangle(frame, (20, 20), (640, 130), (15, 20, 30), -1)
        cv2.rectangle(frame, (20, 20), (640, 130), (90, 180, 255), 2)
        cv2.putText(frame, "ARGUS STACKING ROBOT",
                    (35, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Mobile Mover + AgileX PiPER 6DoF (ros2_control)",
                    (35, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180, 200, 230), 1, cv2.LINE_AA)
        cv2.putText(frame, "QUBO-guided Pick & Place into LD3 ULD",
                    (35, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180, 200, 230), 1, cv2.LINE_AA)
        cv2.putText(frame, f"runtime: {elapsed:6.2f} s",
                    (35, 122), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (140, 200, 230), 1, cv2.LINE_AA)

        # ---- Top-right: cycle progress
        cy = 0
        if cur and "cycle" in cur:
            cy = cur["cycle"]
        cv2.rectangle(frame, (w - 320, 20), (w - 20, 100), (15, 20, 30), -1)
        cv2.rectangle(frame, (w - 320, 20), (w - 20, 100), (60, 200, 90), 2)
        cv2.putText(frame, f"Cycle  {cy} / {total_cycles}",
                    (w - 305, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
        placed = self.placed_so_far(now_wall)
        cv2.putText(frame, f"Placed: {len(placed)} bags",
                    (w - 305, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 255, 220), 1, cv2.LINE_AA)

        # ---- Center-bottom: current phase banner
        phase = cur.get("phase", "idle") if cur else "idle"
        phase_label = PHASE_LABELS.get(phase, phase.upper())
        # Find current bag info
        bag_info = None
        if cur and cur.get("bag_name"):
            bag_info = self.bag_info(cur["bag_name"])
        banner_y = h - 80
        bx, by = w // 2 - 380, banner_y - 30
        bw, bh = 760, 60
        ov = frame.copy()
        cv2.rectangle(ov, (bx, by), (bx + bw, by + bh), (10, 25, 40), -1)
        cv2.addWeighted(ov, 0.82, frame, 0.18, 0, frame)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (60, 220, 255), 2)
        cv2.putText(frame, f"PHASE: {phase_label}",
                    (bx + 15, by + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (60, 220, 255), 2, cv2.LINE_AA)

        # ---- Bag chip (under banner) when carrying
        if bag_info is not None and phase in ("grasp", "attach", "lift", "preplace", "place"):
            cx, cy2 = w // 2 - 340, banner_y + 45
            cw, ch = 680, 50
            cv2.rectangle(frame, (cx, cy2), (cx + cw, cy2 + ch), (12, 18, 26), -1)
            cv2.rectangle(frame, (cx, cy2), (cx + cw, cy2 + ch), (90, 180, 255), 1)
            dims = f"{bag_info['L_mm']}x{bag_info['W_mm']}x{bag_info['H_mm']} mm"
            label = (f"{bag_info['bag_name']:25s}  Class {bag_info['class']:<2s}  "
                     f"Material {bag_info['material']:<3s}  {dims}  {bag_info['mass_kg']:.1f} kg")
            cv2.putText(frame, label, (cx + 14, cy2 + 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 245, 255), 1, cv2.LINE_AA)

        # ---- Bottom-left rolling "Placed in ULD" list
        if placed:
            fx, fy = 30, h - 220 - 26 * min(len(placed), 5)
            fh = 26 * (min(len(placed), 5) + 1) + 12
            cv2.rectangle(frame, (fx, fy), (fx + 460, fy + fh), (12, 18, 26), -1)
            cv2.rectangle(frame, (fx, fy), (fx + 460, fy + fh), (90, 180, 255), 1)
            cv2.putText(frame, "ULD-LD3 STACK (most recent first)",
                        (fx + 10, fy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 180, 255), 1, cv2.LINE_AA)
            for i, e in enumerate(placed[::-1][:5]):
                ly = fy + 20 + 26 * (i + 1)
                cv2.putText(frame,
                            f"#{i + 1}  {e['class']}/{e['material']}  {e['L_mm']}x{e['W_mm']}x{e['H_mm']} mm  ({e['mass_kg']:.1f}kg)",
                            (fx + 12, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 230, 240), 1, cv2.LINE_AA)
        return frame

    def on_image(self, msg):
        if msg.encoding not in ("rgb8", "RGB8", "bgr8", "BGR8"):
            return
        h, w = msg.height, msg.width
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((h, w, 3))
        if msg.encoding.lower() == "rgb8":
            bgr = arr[:, :, ::-1].copy()
        else:
            bgr = arr.copy()
        bgr = self.overlay(bgr)
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
    rclpy.init()
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
