#!/usr/bin/env python3
"""Is the robot actually settling, or standing while vibrating?

test_bridge_stand.py read one frame and saw |lin_acc| = 23.9 m/s^2 (2.4 g) on a robot that
should be at rest reading 9.81. One frame cannot distinguish "still in the transient after
pushing itself off the ground" from "standing but buzzing". This samples over time.
"""

import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

DEFAULT_POS = [0.26, 0.0, -0.52, -0.26, 0.0, 0.52, 0.26, 0.0, -0.52, -0.26, 0.0, 0.52]
KP, KD = 7.5, 0.5
CONTROL_HZ = 520.0


class D(Node):
    def __init__(self):
        super().__init__("test_bridge_settle")
        self.pub = self.create_publisher(Float64MultiArray, "/isaac/joint_command", 10)
        self.state = None
        self.create_subscription(Float64MultiArray, "/isaac/joint_state", self._s, 10)

    def _s(self, m):
        self.state = list(m.data)


def mag(v):
    return sum(x * x for x in v) ** 0.5


def main():
    rclpy.init()
    n = D()
    cmd = Float64MultiArray()
    cmd.data = list(DEFAULT_POS) + [0.0] * 12 + [0.0] * 12 + [KP] * 12 + [KD] * 12

    t0 = time.time()
    while n.state is None and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.state is None:
        print("FAIL: no state")
        return 1

    period = 1.0 / CONTROL_HZ
    print(f"{'t':>5} {'body_z':>8} {'|linacc|':>9} {'linacc_z':>9} {'|angvel|':>9} {'max_err':>8}")
    t0 = time.time()
    window = []
    while time.time() - t0 < 12.0:
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(period)
        el = time.time() - t0
        if len(window) < int(el):
            s = n.state
            lin = s[31:34]
            ang = s[28:31]
            err = max(abs(p - t) for p, t in zip(s[0:12], DEFAULT_POS))
            print(f"{el:5.1f} {s[36]:8.4f} {mag(lin):9.3f} {lin[2]:9.3f} {mag(ang):9.3f} {err:8.4f}")
            window.append((s[36], mag(lin), err))

    # steady state = last 4 samples
    tail = window[-4:]
    zs = [w[0] for w in tail]
    accs = [w[1] for w in tail]
    errs = [w[2] for w in tail]
    print()
    print(f"steady body_z  : mean {statistics.mean(zs):.4f}  spread {max(zs)-min(zs):.5f}")
    print(f"steady |linacc|: mean {statistics.mean(accs):.3f}  (at rest should be ~9.81)")
    print(f"steady max_err : mean {statistics.mean(errs):.4f} rad")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
