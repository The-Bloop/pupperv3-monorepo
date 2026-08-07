#!/usr/bin/env python3
"""Drive isaac_bridge.py the way IsaacHardwareInterface will, and check the robot stands.

This is the real proof of the neutralised-drives + ported-actuator-model design: PhysX
contributes NO PD at all (stiffness=0, damping=0), so if the robot holds its pose the
torque is coming entirely from the ported MuJoCo motor model.

Run from a normal jazzy shell inside pupper-jazzy (python 3.12, real rclpy):
    python3 test_bridge_stand.py
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Canonical order (config/pupper_v3_isaac.yaml joint_order):
# front_r_1/2/3, front_l_1/2/3, back_r_1/2/3, back_l_1/2/3
DEFAULT_POS = [0.26, 0.0, -0.52, -0.26, 0.0, 0.52, 0.26, 0.0, -0.52, -0.26, 0.0, 0.52]
# neural_controller/launch/config.yaml init_kps / init_kds
KP = 7.5
KD = 0.5

CONTROL_HZ = 520.0  # neural_controller/launch/config.yaml controller_manager update_rate
SETTLE_S = 4.0


class Driver(Node):
    def __init__(self):
        super().__init__("test_bridge_stand")
        self.pub = self.create_publisher(Float64MultiArray, "/isaac/joint_command", 10)
        self.state = None
        self.create_subscription(
            Float64MultiArray, "/isaac/joint_state", self._on_state, 10
        )

    def _on_state(self, msg):
        self.state = list(msg.data)


def main():
    rclpy.init()
    n = Driver()

    cmd = Float64MultiArray()
    cmd.data = (
        list(DEFAULT_POS)          # pos_tgt  [0:12]
        + [0.0] * 12               # vel_tgt  [12:24]
        + [0.0] * 12               # ff       [24:36]
        + [KP] * 12                # kp       [36:48]
        + [KD] * 12                # kd       [48:60]
    )
    assert len(cmd.data) == 60

    print("waiting for /isaac/joint_state ...")
    t0 = time.time()
    while n.state is None and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.state is None:
        print("FAIL: no state received from bridge")
        return 1
    print(f"state OK (len={len(n.state)}), body z at start = {n.state[36]:.4f}")

    print(f"commanding default pose @ {CONTROL_HZ:.0f} Hz for {SETTLE_S}s ...")
    period = 1.0 / CONTROL_HZ
    t0 = time.time()
    while time.time() - t0 < SETTLE_S:
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(period)

    # let the last commands land
    for _ in range(50):
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.005)

    s = n.state
    pos = s[0:12]
    quat_xyzw = s[24:28]
    lin_acc = s[31:34]
    body_pos = s[34:37]

    err = [abs(p - t) for p, t in zip(pos, DEFAULT_POS)]
    print()
    print(f"body_pos      : x={body_pos[0]:+.4f} y={body_pos[1]:+.4f} z={body_pos[2]:.4f}")
    print(f"imu quat xyzw : {[round(v, 3) for v in quat_xyzw]}")
    print(f"imu lin_acc   : {[round(v, 3) for v in lin_acc]}   (expect ~+9.81 in z at rest)")
    print(f"joint pos     : {[round(p, 3) for p in pos]}")
    print(f"joint target  : {DEFAULT_POS}")
    print(f"max joint err : {max(err):.4f} rad")
    print()

    # Standing height is 0.16139 (measured); expect some PD droop below that.
    standing = body_pos[2] > 0.12
    imu_live = abs(lin_acc[2]) > 1.0
    print(f"STANDING (body z > 0.12): {standing}")
    print(f"IMU LIVE (|lin_acc.z| > 1): {imu_live}")
    rclpy.shutdown()
    return 0 if (standing and imu_live) else 1


if __name__ == "__main__":
    raise SystemExit(main())
