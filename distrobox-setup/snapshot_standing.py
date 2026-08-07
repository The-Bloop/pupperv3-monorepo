#!/usr/bin/env python3
"""Average a standing robot's state over a window. Works for EITHER backend.

Primary comparable = /joint_states (published by joint_state_broadcaster in both mujoco and
isaac backends) -> 12 joint angles in a fixed canonical order.

Bonus (isaac only): if /isaac/joint_state is present, also report ground-truth body_z from
the wire (state[36]).

Usage: snapshot_standing.py <label>
"""
import statistics
import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

CANON = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]

label = sys.argv[1] if len(sys.argv) > 1 else "?"
rclpy.init()
n = rclpy.create_node("snapshot_standing")
samples = {j: [] for j in CANON}
body_z = []

def on_js(m):
    idx = {name: k for k, name in enumerate(m.name)}
    for j in CANON:
        if j in idx:
            samples[j].append(m.position[idx[j]])

def on_isaac(m):
    d = list(m.data)
    if len(d) >= 37:
        body_z.append(d[36])

n.create_subscription(JointState, "/joint_states", on_js, 50)
n.create_subscription(Float64MultiArray, "/isaac/joint_state", on_isaac, 20)

t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.02)

got = sum(len(v) for v in samples.values())
print(f"[{label}] /joint_states samples: {len(samples[CANON[0]])}  (total {got})")
if len(samples[CANON[0]]) == 0:
    print("FAIL: no /joint_states")
    raise SystemExit(1)
print(f"{'joint':16} {'mean_rad':>10} {'std':>8}")
means = {}
for j in CANON:
    v = samples[j]
    m = statistics.mean(v)
    means[j] = m
    s = statistics.pstdev(v) if len(v) > 1 else 0.0
    print(f"{j:16} {m:10.5f} {s:8.5f}")
if body_z:
    print(f"body_z (isaac wire): mean {statistics.mean(body_z):.5f}  spread {max(body_z)-min(body_z):.5f}")
# machine-readable line for later diff
print("CSV," + label + "," + ",".join(f"{means[j]:.6f}" for j in CANON) +
      ("," + f"{statistics.mean(body_z):.6f}" if body_z else ",nan"))
rclpy.shutdown()
