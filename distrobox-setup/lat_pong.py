#!/usr/bin/env python3
"""Listener for lat_ping.py. Reports one-way DDS transport latency distribution."""
import statistics
import time

import rclpy
from std_msgs.msg import Float64MultiArray

rclpy.init()
n = rclpy.create_node("lat_pong")
lat_ms = []

def cb(m):
    t_recv = time.monotonic_ns()
    t_send = m.data[1]
    lat_ms.append((t_recv - t_send) / 1e6)

n.create_subscription(Float64MultiArray, "/lat/ping", cb, 50)
t0 = time.time()
while time.time() - t0 < 14:
    rclpy.spin_once(n, timeout_sec=0.02)

if not lat_ms:
    print("FAIL: no /lat/ping received")
    raise SystemExit(1)
lat_ms.sort()
n_s = len(lat_ms)
def pct(p):
    return lat_ms[min(n_s - 1, int(p * n_s))]
print(f"one-way DDS latency over {n_s} msgs (CycloneDDS, same host):")
print(f"  mean {statistics.mean(lat_ms):.3f} ms   median {statistics.median(lat_ms):.3f} ms")
print(f"  p50 {pct(0.50):.3f}  p90 {pct(0.90):.3f}  p99 {pct(0.99):.3f}  max {max(lat_ms):.3f} ms")
cyc = 1000.0 / 520.0
print(f"  one 520 Hz control cycle = {cyc:.3f} ms")
print(f"  mean one-way = {statistics.mean(lat_ms) / cyc:.2f} control cycles")
rclpy.shutdown()
