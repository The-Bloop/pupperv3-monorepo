#!/usr/bin/env python3
"""T-Monitor CSV recorder — one run, one sim, one file.

Subscribes /applied_torque (std_msgs/Float64MultiArray, batched physics-rate samples
published by either the MuJoCo plugin or isaac_bridge.py) and writes every sample to
results/t_monitor/<backend>_<timestamp>.csv.

The two simulators never run at once (RUNBOOK "One sim at a time": both spawn a
controller_manager + /robot_description and collide), so this records a single sim per run.
Compare MuJoCo vs Isaac by running each in turn and diffing the two CSVs on sim_time (col 0,
each sim's own clock) — do NOT row-align, the physics rates differ (MuJoCo 10 kHz, Isaac 1 kHz).

Message layout (both sims): layout.dim = [rows=N, cols=1+12]; data = N flattened rows, each
[sim_time, tau_0..tau_11] in canonical joint order.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

N_JOINTS = 12


def _joint_names() -> list[str]:
    """Canonical joint order from pupper_v3_isaac.yaml (single source of truth, shared with
    both emitters). Column labels only; if the share path is unreadable we warn and fall back
    to generic tau_i names so a label lookup never silently corrupts the recorded data."""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory

        cfg = (
            Path(get_package_share_directory("pupper_v3_description"))
            / "description" / "isaac_sim" / "config" / "pupper_v3_isaac.yaml"
        )
        names = list(yaml.safe_load(cfg.read_text())["joint_order"])
        if len(names) != N_JOINTS:
            raise ValueError(f"joint_order has {len(names)} entries, expected {N_JOINTS}")
        return names
    except Exception as e:  # noqa: BLE001 -- labels only; keep recording, warn loudly
        print(f"[torque_recorder] WARNING: could not read joint_order ({e}); "
              f"using generic column names", file=sys.stderr)
        return [f"tau_{i}" for i in range(N_JOINTS)]


class TorqueRecorder(Node):
    def __init__(self, backend: str, out_dir: Path) -> None:
        super().__init__("torque_recorder")
        out_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = out_dir / f"{backend}_{ts}.csv"
        self._file = self._path.open("w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["sim_time", *_joint_names()])
        self._rows = 0
        self._cols = 1 + N_JOINTS

        self.create_subscription(Float64MultiArray, "/applied_torque", self._on_torque, 50)
        # Periodic flush so a Ctrl-C mid-run still leaves a readable file.
        self.create_timer(1.0, lambda: self._file.flush())
        self.get_logger().info(f"T-Monitor recording {backend} -> {self._path}")

    def _on_torque(self, msg: Float64MultiArray) -> None:
        # Reshape via the declared layout; fall back to inferring from data length only if the
        # publisher omitted dims (it should not).
        cols = msg.layout.dim[1].size if len(msg.layout.dim) >= 2 else self._cols
        if cols != self._cols:
            self.get_logger().error(
                f"/applied_torque cols={cols} != {self._cols}; ignoring message")
            return
        data = msg.data
        for i in range(0, len(data), cols):
            row = data[i:i + cols]
            if len(row) != cols:
                break
            self._writer.writerow([f"{v:.9g}" for v in row])
            self._rows += 1

    def destroy_node(self) -> bool:
        self._file.flush()
        self._file.close()
        self.get_logger().info(f"T-Monitor wrote {self._rows} rows -> {self._path}")
        return super().destroy_node()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--backend", required=True, choices=["mujoco", "isaac", "real"],
                   help="Names the output file; which sim this run recorded.")
    p.add_argument("--out-dir", default="results/t_monitor",
                   help="Directory for the CSV (created if missing).")
    args, _ = p.parse_known_args()

    rclpy.init()
    node = TorqueRecorder(args.backend, Path(args.out_dir))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT: the default signal handler already tore down the context. Just flush + close.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
