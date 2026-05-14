#!/usr/bin/env python3
"""
Contest 3 Pick & Place controller.

Orchestrates the demo:
  1. Move arm to home
  2. Move arm to pre-grasp over conveyor
  3. Spawn a bag with DetachableJoint plugin so it auto-attaches to gripper
  4. Lift, swing to ULD side, lower, detach -> bag falls into bin
  5. Return to home, repeat with different bags

Uses:
  - /arm_controller/joint_trajectory topic for arm motion
  - /gripper_controller/joint_trajectory topic for gripper
  - gz service /world/.../create for bag spawn
  - gz topic detach_<bag> for bag release
"""
import json
import math
import os
import random
import subprocess
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


WORLD = "pickplace_world"
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINTS = ["gripper_left"]
GRIPPER_OPEN = 0.038
EVENTS_PATH = "/tmp/arm_events.jsonl"


def log_event(payload):
    payload = dict(payload)
    payload["t"] = time.time()
    try:
        with open(EVENTS_PATH, "a") as f:
            f.write(json.dumps(payload) + "\n")
    except OSError:
        pass
GRIPPER_CLOSED = 0.005

# Joint configurations (joint1..joint6) in radians.
# joint1 rotates the arm around vertical axis (positive = CCW from above).
# joint2 lifts the shoulder (range 0..3.14, larger = arm raised).
# joint3 elbow (range -2.97..0, more negative = more bent).
# joint4/5/6 wrist orientation.
HOME = [0.0, 1.1, -1.4, 0.0, 0.5, 0.0]

# Above the conveyor pickup zone (conveyor center at world x=0.7, y=0)
PREGRASP_OVER_BELT = [0.0, 0.6, -1.5, 0.0, 0.9, 0.0]
GRASP_AT_BELT     = [0.0, 0.9, -1.7, 0.0, 0.8, 0.0]
LIFT_FROM_BELT    = [0.0, 0.5, -1.2, 0.0, 0.7, 0.0]

# Above the ULD bin (bin center at world y=-0.55)
# joint1 negative rotates arm toward -Y
PREPLACE_OVER_ULD = [-1.57, 0.6, -1.4, 0.0, 0.8, 0.0]
PLACE_INTO_ULD    = [-1.57, 0.9, -1.6, 0.0, 0.75, 0.0]
LIFT_FROM_ULD     = [-1.57, 0.5, -1.2, 0.0, 0.7, 0.0]


# Demo bags are deliberately small cubic boxes (~10 cm) sized for the PiPER
# 4 cm gripper stroke. Production scale (22-32 kg full-size luggage) is
# Stage2 work using the Mobile Mover full payload and a heavy-duty arm.
# (label, L, W, H, mass, ambient_rgb, diffuse_rgb)
BAG_TYPES = [
    ("cube_red",     0.10, 0.10, 0.10, 0.6, (0.30, 0.05, 0.05), (0.85, 0.18, 0.18)),
    ("cube_navy",    0.10, 0.10, 0.10, 0.6, (0.05, 0.08, 0.22), (0.15, 0.25, 0.60)),
    ("cube_olive",   0.10, 0.10, 0.10, 0.6, (0.18, 0.22, 0.10), (0.45, 0.55, 0.22)),
    ("cube_silver",  0.10, 0.10, 0.10, 0.6, (0.30, 0.30, 0.34), (0.75, 0.78, 0.82)),
    ("cube_amber",   0.10, 0.10, 0.10, 0.6, (0.30, 0.20, 0.05), (0.85, 0.55, 0.10)),
]


def gz(*args, timeout=5.0):
    try:
        r = subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout)
        return r.stdout, r.stderr
    except subprocess.TimeoutExpired:
        return "", "<TIMEOUT>"


def bag_sdf_with_attach(name: str, btype, attach_to_link: str = "link6") -> str:
    """Bag SDF including DetachableJoint plugin auto-attaching to gripper."""
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
        '<surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>'
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
        '<plugin filename="gz-sim-detachable-joint-system" '
        'name="gz::sim::systems::DetachableJoint">'
        '<parent_link>body</parent_link>'
        f'<child_model>piper</child_model>'
        f'<child_link>{attach_to_link}</child_link>'
        f'<detach_topic>/{name}/detach</detach_topic>'
        '</plugin>'
        '</model>'
        '</sdf>'
    )


def get_link_pose(model: str, link: str):
    """Return (x, y, z) world position of a link, parsed from `gz model -m ...`.
    Returns None on failure."""
    out, _ = gz("model", "-m", model, timeout=4.0)
    if not out:
        return None
    lines = out.splitlines()
    in_target = False
    saw_pose_header = False
    for i, line in enumerate(lines):
        if "- Name:" in line and line.strip().endswith(link):
            in_target = True
            saw_pose_header = False
            continue
        if in_target:
            if "- Name:" in line:
                in_target = False
                continue
            # Find the world Pose line (the second "Pose [ XYZ" inside link)
            if "Pose [ XYZ" in line and "Inertial" not in line:
                saw_pose_header = True
                continue
            if saw_pose_header and line.strip().startswith("["):
                # parse "[x y z]"
                nums = line.strip().strip("[]").split()
                if len(nums) >= 3:
                    try:
                        return float(nums[0]), float(nums[1]), float(nums[2])
                    except ValueError:
                        return None
    return None


def spawn_bag(name: str, btype, x: float, y: float, z: float, yaw: float = 0.0):
    sdf = bag_sdf_with_attach(name, btype)
    qw = math.cos(yaw * 0.5)
    qz = math.sin(yaw * 0.5)
    req = (
        f"sdf: '{sdf}' "
        f'name: "{name}" '
        f'pose: {{ position: {{x: {x}, y: {y}, z: {z}}} '
        f'orientation: {{w: {qw}, z: {qz}}} }}'
    )
    out, err = gz("service", "-s", f"/world/{WORLD}/create",
                  "--reqtype", "gz.msgs.EntityFactory",
                  "--reptype", "gz.msgs.Boolean",
                  "--timeout", "3000",
                  "--req", req)
    if "data: true" not in out:
        print(f"[spawn_bag] FAIL {name}: {err[:300]}")
        return False
    return True


def detach_bag(name: str):
    gz("topic", "-t", f"/{name}/detach",
       "-m", "gz.msgs.Empty",
       "-p", "", timeout=2.0)


def remove_bag(name: str):
    req = f'name: "{name}" type: MODEL'
    gz("service", "-s", f"/world/{WORLD}/remove",
       "--reqtype", "gz.msgs.Entity",
       "--reptype", "gz.msgs.Boolean",
       "--timeout", "1500",
       "--req", req, timeout=3.0)


class PickPlace(Node):
    def __init__(self):
        super().__init__("pickplace_controller")
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.grip_pub = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        time.sleep(2.0)  # let publishers connect
        self.get_logger().info("PickPlace controller ready")

    def move_arm(self, positions, duration_s=2.5):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s - int(duration_s)) * 1e9))
        msg.points = [pt]
        self.arm_pub.publish(msg)
        time.sleep(duration_s + 0.3)

    def move_gripper(self, pos, duration_s=0.5):
        msg = JointTrajectory()
        msg.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [pos]
        pt.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s - int(duration_s)) * 1e9))
        msg.points = [pt]
        self.grip_pub.publish(msg)
        time.sleep(duration_s + 0.2)


def main():
    # Reset event log for the annotator
    try:
        open(EVENTS_PATH, "w").close()
    except OSError:
        pass
    # Disable rclpy's automatic SIGINT handler so subprocess.run() in gz()
    # cannot accidentally shutdown the context mid-demo.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PickPlace()
    time.sleep(0.5)
    cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    log_event({"phase": "init", "total_cycles": cycles})

    node.get_logger().info("=> HOME (initial)")
    log_event({"phase": "home_initial"})
    node.move_arm(HOME, duration_s=3.0)
    node.move_gripper(GRIPPER_OPEN, duration_s=0.5)

    time.sleep(1.0)

    bag_id = 0
    for cycle in range(cycles):
        btype = BAG_TYPES[cycle % len(BAG_TYPES)]
        name = f"bag_{bag_id:03d}_{btype[0]}"
        bag_id += 1
        label = btype[0]
        if label.startswith("hardX"): bag_class = "HX"
        elif label.startswith("hard"): bag_class = "H"
        elif label.startswith("softX"): bag_class = "SX"
        else: bag_class = "S"
        color_tok = label.rsplit("_", 1)[-1]
        mat_map = {"red": "FAB", "navy": "PLA", "olive": "FAB", "silver": "MET",
                   "beige": "FAB", "black": "PLA", "metal": "MET",
                   "leather": "LEA", "teal": "PLA", "mustard": "FAB"}
        bag_material = mat_map.get(color_tok, "MIX")
        log_event({"phase": "cycle_start", "cycle": cycle + 1, "total": cycles,
                   "bag_name": name, "class": bag_class, "material": bag_material,
                   "L_mm": int(btype[1]*1000), "W_mm": int(btype[2]*1000),
                   "H_mm": int(btype[3]*1000), "mass_kg": btype[4]})

        node.get_logger().info(f"[cycle {cycle+1}/{cycles}] => PREGRASP_OVER_BELT (bag={name})")
        log_event({"phase": "pregrasp", "cycle": cycle + 1, "bag_name": name})
        node.move_arm(PREGRASP_OVER_BELT, duration_s=2.0)

        node.get_logger().info("=> GRASP")
        log_event({"phase": "grasp", "cycle": cycle + 1, "bag_name": name})
        node.move_arm(GRASP_AT_BELT, duration_s=1.4)

        node.move_gripper(GRIPPER_CLOSED, duration_s=0.4)
        time.sleep(0.2)

        # Spawn bag at link6 with a forward+down offset so the visible center
        # of the bag is in front of the gripper, not occluded by the wrist mesh.
        link_pose = get_link_pose("piper", "link6")
        if link_pose is None:
            node.get_logger().warn("link6 pose unavailable; falling back to hardcoded spawn")
            spawn_x, spawn_y, spawn_z = 0.55, 0.0, 0.70
        else:
            spawn_x, spawn_y, spawn_z = link_pose
            # 10 cm cube; place it just below the wrist so it sits at the
            # finger tips and is fully visible outside the gripper mesh.
            spawn_z -= 0.18
        node.get_logger().info(f"=> SPAWN+ATTACH at ({spawn_x:.3f},{spawn_y:.3f},{spawn_z:.3f})")
        log_event({"phase": "attach", "cycle": cycle + 1, "bag_name": name,
                   "spawn_xyz": [spawn_x, spawn_y, spawn_z]})
        ok = spawn_bag(name, btype, spawn_x, spawn_y, spawn_z, yaw=0.0)
        if not ok:
            node.get_logger().error(f"Bag {name} spawn failed; skipping cycle")
            log_event({"phase": "attach_fail", "cycle": cycle + 1, "bag_name": name})
            continue
        time.sleep(0.4)

        node.get_logger().info("=> LIFT_FROM_BELT")
        log_event({"phase": "lift", "cycle": cycle + 1, "bag_name": name})
        node.move_arm(LIFT_FROM_BELT, duration_s=1.6)

        node.get_logger().info("=> PREPLACE_OVER_ULD")
        log_event({"phase": "preplace", "cycle": cycle + 1, "bag_name": name})
        node.move_arm(PREPLACE_OVER_ULD, duration_s=2.4)

        node.get_logger().info("=> PLACE_INTO_ULD")
        log_event({"phase": "place", "cycle": cycle + 1, "bag_name": name})
        node.move_arm(PLACE_INTO_ULD, duration_s=1.6)

        node.move_gripper(GRIPPER_OPEN, duration_s=0.4)
        time.sleep(0.1)

        node.get_logger().info("=> DETACH (bag drops)")
        log_event({"phase": "detach", "cycle": cycle + 1, "bag_name": name})
        detach_bag(name)
        time.sleep(0.6)

        # Retract
        node.get_logger().info("=> LIFT_FROM_ULD")
        node.move_arm(LIFT_FROM_ULD, duration_s=1.5)

        # Back to home for next cycle
        node.get_logger().info("=> HOME")
        node.move_arm(HOME, duration_s=2.0)

    node.get_logger().info("DONE all cycles")
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
