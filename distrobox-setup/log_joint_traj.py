#!/usr/bin/env python3
"""Log per-joint mean angle in successive windows to reveal slow settling/drift.

Prints one CSV row per window: t_mid, then 12 joint means (canonical order).
Usage: log_joint_traj.py <label> <total_seconds> <window_seconds>
"""
import statistics
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

CANON = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]
label = sys.argv[1] if len(sys.argv) > 1 else "?"
total = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
win = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

rclpy.init()
n = rclpy.create_node("log_joint_traj")
buf = {j: [] for j in CANON}

def on_js(m):
    idx = {name: k for k, name in enumerate(m.name)}
    for j in CANON:
        if j in idx:
            buf[j].append(m.position[idx[j]])

n.create_subscription(JointState, "/joint_states", on_js, 50)
print("traj," + label + ",t," + ",".join(CANON), flush=True)
t_start = time.time()
w_start = t_start
while time.time() - t_start < total:
    rclpy.spin_once(n, timeout_sec=0.02)
    if time.time() - w_start >= win:
        tmid = time.time() - t_start
        means = [statistics.mean(buf[j]) if buf[j] else float("nan") for j in CANON]
        print("traj," + label + f",{tmid:.1f}," + ",".join(f"{v:.4f}" for v in means), flush=True)
        buf = {j: [] for j in CANON}
        w_start = time.time()
rclpy.shutdown()
