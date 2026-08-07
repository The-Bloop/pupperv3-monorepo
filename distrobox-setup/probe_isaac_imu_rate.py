#!/usr/bin/env python3
"""Confirm the accelerometer is quiet now that ground restitution=0, and report loop rates.

State wire layout: [pos x12, vel x12, imu x10, body_pos x3, body_vel x3]
  imu(10) = orientation(x,y,z,w) + ang_vel(xyz) + lin_acc(xyz)
  -> imu block starts at index 24; lin_acc = indices 31,32,33
"""
import math
import statistics
import time

import rclpy
from std_msgs.msg import Float64MultiArray

rclpy.init()
n = rclpy.create_node("probe_isaac_imu_rate")
st_times, cmd_times = [], []
accels = []
last = {}

def on_state(m):
    d = list(m.data)
    st_times.append(time.monotonic())
    ax, ay, az = d[31], d[32], d[33]
    accels.append(math.sqrt(ax * ax + ay * ay + az * az))
    last["state"] = d

def on_cmd(m):
    cmd_times.append(time.monotonic())

n.create_subscription(Float64MultiArray, "/isaac/joint_state", on_state, 50)
n.create_subscription(Float64MultiArray, "/isaac/joint_command", on_cmd, 50)

t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.02)

def hz(ts):
    if len(ts) < 2:
        return 0.0
    return (len(ts) - 1) / (ts[-1] - ts[0])

print(f"samples: state={len(st_times)} cmd={len(cmd_times)}")
print(f"/isaac/joint_state  rate: {hz(st_times):7.1f} Hz")
print(f"/isaac/joint_command rate: {hz(cmd_times):7.1f} Hz")
if accels:
    print(f"|lin_acc|  mean {statistics.mean(accels):.4f}  std {statistics.pstdev(accels):.4f}  "
          f"min {min(accels):.4f}  max {max(accels):.4f}  (expect ~9.81, low std if not bouncing)")
if "state" in last:
    d = last["state"]
    print(f"orientation (x,y,z,w): {d[24]:+.3f} {d[25]:+.3f} {d[26]:+.3f} {d[27]:+.3f}")
    print(f"ang_vel  (x,y,z): {d[28]:+.4f} {d[29]:+.4f} {d[30]:+.4f}")
    print(f"body_pos (x,y,z): {d[34]:+.4f} {d[35]:+.4f} {d[36]:+.4f}")
    print(f"body_vel (x,y,z): {d[37]:+.4f} {d[38]:+.4f} {d[39]:+.4f}")
rclpy.shutdown()
