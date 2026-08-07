#!/usr/bin/env python3
"""Watch the robot while neural_controller drives Isaac Sim.

Taps the wire between IsaacHardwareInterface and isaac_bridge.py so we can see what the
POLICY is actually commanding -- in particular the live kp/kd, which is the whole reason
this needed a real ros2_control SystemInterface rather than the JointState topic bridge.
"""

import statistics
import time

import rclpy
from std_msgs.msg import Float64MultiArray

rclpy.init()
n = rclpy.create_node("watch_nc_isaac")
st, cmd = {}, {}
n.create_subscription(Float64MultiArray, "/isaac/joint_state", lambda m: st.update(d=list(m.data)), 10)
n.create_subscription(Float64MultiArray, "/isaac/joint_command", lambda m: cmd.update(d=list(m.data)), 10)

t0 = time.time()
while "d" not in st and time.time() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.1)
t0 = time.time()
while "d" not in cmd and time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.1)

print(f"state flowing: {'d' in st} | command flowing: {'d' in cmd}")
if "d" not in cmd:
    print("FAIL: nothing on /isaac/joint_command -- the plugin's write() is not reaching the bridge")
    raise SystemExit(1)
print()
print(f"{'t':>5} {'body_z':>8} {'kp[0]':>7} {'kd[0]':>7} {'postgt[0]':>10} {'jointpos[0]':>11}")

zs, kps = [], []
t0 = time.time()
last = -1.0
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.05)
    el = time.time() - t0
    if el - last >= 2.0:
        last = el
        s, c = st.get("d"), cmd.get("d")
        if s and c:
            # wire layout: cmd = [pos_tgt x12, vel_tgt x12, ff x12, kp x12, kd x12]
            #              state = [pos x12, vel x12, imu x10, body_pos x3, body_vel x3]
            print(f"{el:5.1f} {s[36]:8.4f} {c[36]:7.2f} {c[48]:7.2f} {c[0]:10.3f} {s[0]:11.3f}")
            zs.append(s[36])
            kps.append(c[36])

print()
if zs:
    print(f"body_z  mean {statistics.mean(zs):.4f}  spread {max(zs) - min(zs):.5f}")
    print(f"kp[0]   mean {statistics.mean(kps):.2f}  (policy drives this live: 7.5 init, "
          f"kps*gain_multiplier running, 0 on estop)")
    print(f"STANDING under neural_controller: {statistics.mean(zs) > 0.12}")
rclpy.shutdown()
