#!/usr/bin/env python3
"""
Bag spawner + belt driver for the BHS demo.

- Drives the conveyor belt by publishing track_cmd_vel to TrackController.
- Spawns randomized bags onto the upstream end at regular intervals.
- Bags are physics-driven; the belt surface (mu2=150, fdir1=0 1 0) pushes
  them along +X via TrackController.
- Cleans up bags that have left the camera frame (x > 5.5 or z < 0).
"""
import json
import math
import random
import subprocess
import sys
import time
import threading
from dataclasses import dataclass
from typing import List


# JSONL file the annotator reads to overlay detections.
EVENTS_PATH = "/tmp/bag_events.jsonl"


# Mock "DB matched" entries, sampled to look like real Samsonite/RIMOWA-style hits.
DB_BRANDS = {
    "hardS_black":  ("Samsonite Cosmolite S", 0.992),
    "hardM_silver": ("Rimowa Original Check-In M", 0.987),
    "hardL_navy":   ("Travelpro Maxlite 5 L", 0.981),
    "softS_olive":  ("Briggs & Riley Baseline S", 0.964),
    "softM_red":    ("Tumi Voyageur M", 0.972),
    "softL_beige":  ("Eagle Creek Cargo Hauler L", 0.958),
    "hardX_metal":  ("Zero Halliburton Geo Aluminum", 0.996),
    "softX_leather":("Tumi Alpha 3 Leather", 0.969),
    "hardM_teal":   ("Away Bigger Carry-On", 0.989),
    "softM_mustard":("Patagonia Black Hole 70L", 0.961),
}

# 4-class × 7-material mapping derived from bag label
def classify(label: str):
    if label.startswith("hardX"): cls = "HX"
    elif label.startswith("hard"): cls = "H"
    elif label.startswith("softX"): cls = "SX"
    else: cls = "S"
    mat_map = {
        "black": "PLA", "silver": "MET", "navy": "PLA",
        "olive": "FAB", "red": "FAB", "beige": "FAB",
        "metal": "MET", "leather": "LEA", "teal": "PLA",
        "mustard": "FAB",
    }
    color = label.split("_", 1)[1] if "_" in label else ""
    color = color[1:] if color and color[0].isupper() == False and "M_" in label or "S_" in label or "L_" in label or "X_" in label else color
    # simpler: take last token after last underscore
    color_tok = label.rsplit("_", 1)[-1]
    mat = mat_map.get(color_tok, "MIX")
    return cls, mat


BELT_SPEED = 0.4167  # m/s (25 m/min)
BELT_TOP_Z = 0.54    # ~ belt center 0.50 + half thickness 0.04
BELT_X_START = 0.30  # upstream end relative to world (conveyor centered at x=2 with length 4 -> 0..4)
BAG_SPAWN_INTERVAL = 2.4
DESPAWN_X = 5.8
DESPAWN_Z = 0.10
WORLD = "bhs_world"
BELT_CMD_TOPIC = "/model/bhs_conveyor/link/belt/track_cmd_vel"


# (label, L, W, H, mass, ambient_rgb, diffuse_rgb)
BAG_TYPES = [
    ("hardS_black",  0.55, 0.36, 0.22, 6.0,  (0.04, 0.04, 0.04), (0.10, 0.10, 0.10)),
    ("hardM_silver", 0.65, 0.42, 0.26, 8.5,  (0.30, 0.30, 0.34), (0.62, 0.62, 0.66)),
    ("hardL_navy",   0.75, 0.48, 0.30, 11.0, (0.05, 0.08, 0.22), (0.10, 0.18, 0.45)),
    ("softS_olive",  0.45, 0.32, 0.20, 4.2,  (0.18, 0.20, 0.10), (0.36, 0.40, 0.20)),
    ("softM_red",    0.60, 0.40, 0.28, 6.8,  (0.30, 0.06, 0.06), (0.70, 0.13, 0.13)),
    ("softL_beige",  0.72, 0.46, 0.32, 9.2,  (0.30, 0.25, 0.18), (0.62, 0.52, 0.38)),
    ("hardX_metal",  0.50, 0.34, 0.18, 7.4,  (0.30, 0.32, 0.36), (0.78, 0.80, 0.84)),
    ("softX_leather",0.55, 0.38, 0.24, 5.0,  (0.18, 0.10, 0.05), (0.36, 0.22, 0.12)),
    ("hardM_teal",   0.60, 0.40, 0.24, 7.2,  (0.05, 0.22, 0.20), (0.10, 0.45, 0.42)),
    ("softM_mustard",0.55, 0.38, 0.26, 6.0,  (0.30, 0.22, 0.05), (0.70, 0.52, 0.12)),
]


def gz(*args, timeout=4.0):
    try:
        r = subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout)
        return r.stdout
    except subprocess.TimeoutExpired:
        return ""


def bag_sdf(name: str, btype) -> str:
    """Return a single-line SDF string (no newlines, to satisfy protobuf text
    parser which forbids string literals crossing line boundaries)."""
    label, L, W, H, mass, amb, dif = btype
    ar, ag, ab = amb
    dr, dg, db = dif
    ixx = (1.0/12.0) * mass * (W*W + H*H)
    iyy = (1.0/12.0) * mass * (L*L + H*H)
    izz = (1.0/12.0) * mass * (L*L + W*W)
    return (
        '<?xml version="1.0"?>'
        '<sdf version="1.10">'
        f'<model name="{name}">'
        '<link name="body">'
        '<inertial>'
        f'<mass>{mass}</mass>'
        f'<inertia><ixx>{ixx:.6f}</ixx><ixy>0</ixy><ixz>0</ixz>'
        f'<iyy>{iyy:.6f}</iyy><iyz>0</iyz>'
        f'<izz>{izz:.6f}</izz></inertia>'
        '</inertial>'
        '<collision name="col">'
        f'<geometry><box><size>{L} {W} {H}</size></box></geometry>'
        '<surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2><fdir1>0 1 0</fdir1></ode></friction></surface>'
        '</collision>'
        '<visual name="vis">'
        f'<geometry><box><size>{L} {W} {H}</size></box></geometry>'
        '<material>'
        f'<ambient>{ar} {ag} {ab} 1</ambient>'
        f'<diffuse>{dr} {dg} {db} 1</diffuse>'
        '<specular>0.30 0.30 0.30 1</specular>'
        '</material>'
        '</visual>'
        '</link>'
        '</model>'
        '</sdf>'
    )


def spawn_bag(name: str, btype, x: float, y: float, z: float, yaw: float):
    sdf = bag_sdf(name, btype)
    qw = math.cos(yaw * 0.5)
    qz = math.sin(yaw * 0.5)
    req = (
        f"sdf: '{sdf}' "
        f"name: \"{name}\" "
        f"pose: {{ position: {{x: {x}, y: {y}, z: {z}}} "
        f"orientation: {{w: {qw}, z: {qz}}} }}"
    )
    out = gz("service", "-s", f"/world/{WORLD}/create",
             "--reqtype", "gz.msgs.EntityFactory",
             "--reptype", "gz.msgs.Boolean",
             "--timeout", "1500",
             "--req", req)
    return "true" in out


def remove_bag(name: str):
    req = f'name: "{name}" type: MODEL'
    gz("service", "-s", f"/world/{WORLD}/remove",
       "--reqtype", "gz.msgs.Entity",
       "--reptype", "gz.msgs.Boolean",
       "--timeout", "800",
       "--req", req, timeout=2.0)


def drive_belt(speed: float):
    """Persistently publish belt cmd_vel via gz topic pub."""
    # gz topic pub publishes once. Use --print-output and short interval to
    # publish a few times to ensure delivery before subscribers exist.
    for _ in range(8):
        gz("topic", "-t", BELT_CMD_TOPIC,
           "-m", "gz.msgs.Double",
           "-p", f"data: {speed}",
           timeout=1.5)
        time.sleep(0.25)


def get_pose_via_state(name: str):
    """Best-effort: returns None (we just track by spawn time)."""
    return None


@dataclass
class ActiveBag:
    name: str
    spawn_t: float
    btype: tuple


def main():
    print("[bag_spawner] starting (assuming world already running)", flush=True)

    # Reset events file for the annotator to consume.
    with open(EVENTS_PATH, "w") as f:
        f.write("")
    spawn_t0 = time.time()

    # Drive belt continuously in a background thread
    stop = threading.Event()
    def belt_driver():
        while not stop.is_set():
            gz("topic", "-t", BELT_CMD_TOPIC,
               "-m", "gz.msgs.Double",
               "-p", f"data: {BELT_SPEED}",
               timeout=1.5)
            time.sleep(0.5)
    t_belt = threading.Thread(target=belt_driver, daemon=True)
    t_belt.start()

    active: List[ActiveBag] = []
    bag_id = 0
    next_spawn_t = time.time() + 1.0

    try:
        while True:
            now = time.time()
            if now >= next_spawn_t:
                btype = random.choice(BAG_TYPES)
                name = f"bag_{bag_id:03d}_{btype[0]}"
                y = random.uniform(-0.10, 0.10)
                yaw = random.uniform(-0.20, 0.20)
                z = BELT_TOP_Z + btype[3] * 0.5 + 0.02
                if spawn_bag(name, btype, BELT_X_START, y, z, yaw):
                    active.append(ActiveBag(name=name, spawn_t=now, btype=btype))
                    cls, mat = classify(btype[0])
                    brand, score = DB_BRANDS.get(btype[0], ("(no match)", 0.0))
                    # Time the bag will pass under the sensor frame (x=2.0)
                    sensor_eta = BELT_X_START + 0.0  # estimate
                    travel_dist = 2.0 - BELT_X_START  # from start to sensor at x=2.0
                    sensor_t = now + (travel_dist / BELT_SPEED)
                    event = {
                        "spawn_t": now,
                        "detect_t": sensor_t,
                        "name": name,
                        "class": cls,
                        "material": mat,
                        "L_mm": int(btype[1] * 1000),
                        "W_mm": int(btype[2] * 1000),
                        "H_mm": int(btype[3] * 1000),
                        "mass_kg": btype[4],
                        "db_brand": brand,
                        "db_score": score,
                    }
                    with open(EVENTS_PATH, "a") as f:
                        f.write(json.dumps(event) + "\n")
                    print(f"[bag_spawner] spawned {name} at y={y:+.2f} -> {cls}/{mat}", flush=True)
                else:
                    print(f"[bag_spawner] FAILED to spawn {name}", flush=True)
                bag_id += 1
                next_spawn_t = now + BAG_SPAWN_INTERVAL

            # cleanup old bags by age (they should have traveled past camera by now)
            keep = []
            for b in active:
                age = now - b.spawn_t
                if age > 16.0:  # after ~16s the bag must have left frame
                    remove_bag(b.name)
                else:
                    keep.append(b)
            active = keep
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("[bag_spawner] stopping", flush=True)
        stop.set()
        for b in active:
            remove_bag(b.name)


if __name__ == "__main__":
    main()
