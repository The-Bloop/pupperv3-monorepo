#!/usr/bin/env python3
"""One-way DDS latency probe. Publishes [seq, t_send_ns] on /lat/ping at 520 Hz.

CLOCK_MONOTONIC is host-wide (distrobox shares the kernel), so the listener in another
process -- even under Isaac's py3.11 -- can subtract t_send from its own monotonic_ns and
get a true one-way transport time. Same rmw/CycloneDDS C layer the real command/state path
uses, so the number transfers.
"""
import time

import rclpy
from std_msgs.msg import Float64MultiArray

rclpy.init()
n = rclpy.create_node("lat_ping")
p = n.create_publisher(Float64MultiArray, "/lat/ping", 10)
seq = 0.0
t0 = time.time()
# 520 Hz to match the controller_manager write() cadence.
while time.time() - t0 < 12:
    m = Float64MultiArray()
    m.data = [seq, float(time.monotonic_ns())]
    p.publish(m)
    seq += 1.0
    time.sleep(1.0 / 520.0)
rclpy.shutdown()
