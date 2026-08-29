#!/usr/bin/env python3
"""T-Monitor plotter — graph applied joint torque from one or more CSVs.

Reads the CSVs written by torque_recorder.py (header: sim_time + 12 canonical joint
names) and draws a 4x3 grid of subplots, one per joint, tau vs sim_time.

Pass two files (a MuJoCo run and an Isaac run) to overlay them for the motor-model
comparison. The two sims run at different physics rates (MuJoCo 10 kHz, Isaac 1 kHz)
and each carries its OWN sim_time clock starting at ~0, so the x-axes line up directly
on sim_time -- do NOT row-align. This is an offline analysis tool; run it on the host
or in the container, wherever matplotlib is installed.

    torque_plot.py results/t_monitor/mujoco_20260829_101500.csv
    torque_plot.py results/t_monitor/mujoco_*.csv results/t_monitor/isaac_*.csv
    torque_plot.py <csv> --save results/t_monitor/compare.png     # headless
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

# Column 0 is the time axis; the rest are per-joint torques.
TIME_COL = "sim_time"


def load(path: Path) -> tuple[list[float], dict[str, list[float]]]:
    """Return (sim_time, {joint_name: [tau...]}) from one T-Monitor CSV.

    No fallback parsing: a malformed file should fail loud, not plot garbage."""
    with path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        if header[0] != TIME_COL:
            sys.exit(f"{path}: first column is '{header[0]}', expected '{TIME_COL}' "
                     f"-- not a T-Monitor CSV")
        joints = header[1:]
        t: list[float] = []
        cols: dict[str, list[float]] = {j: [] for j in joints}
        for row in reader:
            if len(row) != len(header):
                print(f"{path}: skipping short row (len {len(row)} != {len(header)})",
                      file=sys.stderr)
                continue
            t.append(float(row[0]))
            for j, v in zip(joints, row[1:]):
                cols[j].append(float(v))
    if not t:
        sys.exit(f"{path}: no data rows")
    return t, cols


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", nargs="+", type=Path,
                   help="One or more T-Monitor CSVs. Multiple files overlay on shared axes.")
    p.add_argument("--save", type=Path,
                   help="Write PNG here instead of opening a window (forces headless backend).")
    args = p.parse_args()

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = [(path, *load(path)) for path in args.csv]

    # Joint set/order comes from the first file; warn if another file disagrees so a
    # mismatched pair is never silently plotted against the wrong subplot.
    joints = list(runs[0][2].keys())
    for path, _t, cols in runs[1:]:
        if list(cols.keys()) != joints:
            print(f"WARNING: {path} joint order differs from {runs[0][0]}; "
                  f"plotting by name, columns may not correspond", file=sys.stderr)

    ncols = 3
    nrows = (len(joints) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 2.4 * nrows),
                             sharex=True, squeeze=False)
    flat = [ax for r in axes for ax in r]

    for ax, joint in zip(flat, joints):
        for path, t, cols in runs:
            if joint in cols:
                ax.plot(t, cols[joint], linewidth=0.6, label=path.stem)
        ax.set_title(joint, fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=7)
    for ax in flat[len(joints):]:
        ax.axis("off")

    for ax in axes[-1]:
        ax.set_xlabel(TIME_COL, fontsize=8)
    for r in axes:
        r[0].set_ylabel("tau (Nm)", fontsize=8)

    if len(runs) > 1:
        flat[0].legend(fontsize=7, loc="best")
    fig.suptitle("T-Monitor applied joint torque"
                 + (f"  ({len(runs)} runs)" if len(runs) > 1 else ""), fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.98))

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=120)
        print(f"wrote {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
