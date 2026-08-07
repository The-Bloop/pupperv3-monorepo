#!/usr/bin/env python3
"""Assert actuator_model.py matches pupperv3_mujoco_sim's actuator_model.hpp exactly.

The Isaac cross-check is only meaningful if both simulators apply the same actuator model.
This compiles the REAL C++ header (not a transcription of it), runs both implementations
over the same grid, and diffs. Any drift fails here rather than showing up as a bogus
"Isaac and MuJoCo disagree" result later.

Run with any python3 that has a C++ compiler available:
    python3 test_actuator_model_parity.py
"""

from __future__ import annotations

import itertools
import subprocess
import sys
import tempfile
from pathlib import Path

from actuator_model import ActuatorParams, PIDActuatorModel

HERE = Path(__file__).parent.resolve()
# isaac_sim -> description -> pupper_v3_description -> src
MUJOCO_LIB = (HERE / "../../../pupperv3_mujoco_sim/lib").resolve()

# pupper_v3_mujoco.ros2_control.xacro:10-14
BUS_VOLTAGE = 24.0
KT = 0.04
PHASE_RESISTANCE = 0.7
SATURATION_TORQUE = 4.5
SOFTWARE_TORQUE_LIMIT = 3.0

DRIVER = r"""
#include <cstdio>
#include "actuator_model.hpp"

int main(int argc, char** argv) {
    // kp, kd are overwritten per-call by the 7-arg run(), so seed them at 0.
    ActuatorParams params(0.0, 0.0, %(bus)s, %(kt)s, %(res)s, %(sat)s, %(soft)s);
    PIDActuatorModel model(params);
    double kp, kd, pos, pos_t, vel, vel_t, ff;
    while (scanf("%%lf %%lf %%lf %%lf %%lf %%lf %%lf",
                 &kp, &kd, &pos, &pos_t, &vel, &vel_t, &ff) == 7) {
        printf("%%.17g\n", model.run(kp, kd, pos, pos_t, vel, vel_t, ff));
    }
    return 0;
}
""" % {
    "bus": BUS_VOLTAGE, "kt": KT, "res": PHASE_RESISTANCE,
    "sat": SATURATION_TORQUE, "soft": SOFTWARE_TORQUE_LIMIT,
}


def build_cpp_driver(tmpdir: Path) -> Path:
    src = tmpdir / "driver.cpp"
    src.write_text(DRIVER)
    exe = tmpdir / "driver"
    subprocess.run(
        ["g++", "-O0", "-std=c++17", f"-I{MUJOCO_LIB}", str(src), "-o", str(exe)],
        check=True,
    )
    return exe


def grid():
    """Spans the normal envelope plus the pathological cases.

    velocity 700 is deliberate: |kt*vel| = 28 > bus_voltage 24, so emf_limited_torque goes
    negative and std::clamp is invoked with lo > hi (UB). That is the one case where a
    min/max spelling of clamp diverges from libstdc++, so it must be covered.
    """
    kps = [0.0, 5.0, 7.5, 10.0]
    kds = [0.0, 0.5, 1.0]
    errs = [-2.0, -0.05, 0.0, 0.05, 2.0]
    vels = [-700.0, -30.0, -1.0, 0.0, 1.0, 30.0, 600.0, 700.0]
    ffs = [-5.0, 0.0, 5.0]
    for kp, kd, err, vel, ff in itertools.product(kps, kds, errs, vels, ffs):
        # position fixed at 0.0, so position_target == err
        yield (kp, kd, 0.0, err, vel, 0.0, ff)


def main() -> int:
    cases = list(grid())
    print(f"grid: {len(cases)} cases")

    with tempfile.TemporaryDirectory() as td:
        exe = build_cpp_driver(Path(td))
        stdin = "\n".join(" ".join(repr(v) for v in c) for c in cases) + "\n"
        out = subprocess.run([str(exe)], input=stdin, capture_output=True, text=True, check=True)
    cpp_vals = [float(x) for x in out.stdout.split()]

    if len(cpp_vals) != len(cases):
        print(f"FAIL: C++ returned {len(cpp_vals)} values for {len(cases)} cases")
        return 1

    params = ActuatorParams(
        kp=0.0, kd=0.0, bus_voltage=BUS_VOLTAGE, kt=KT,
        phase_resistance=PHASE_RESISTANCE, saturation_torque=SATURATION_TORQUE,
        software_torque_limit=SOFTWARE_TORQUE_LIMIT,
    )
    model = PIDActuatorModel(params)

    worst = 0.0
    fails = []
    for case, cpp_val in zip(cases, cpp_vals):
        py_val = model.run(*case)
        diff = abs(py_val - cpp_val)
        worst = max(worst, diff)
        if diff > 1e-9:
            fails.append((case, cpp_val, py_val, diff))

    if fails:
        print(f"FAIL: {len(fails)}/{len(cases)} cases diverge")
        for case, c, p, d in fails[:10]:
            print(f"  kp={case[0]} kd={case[1]} pos_t={case[3]} vel={case[4]} ff={case[6]}")
            print(f"    cpp={c!r}  py={p!r}  diff={d!r}")
        return 1

    print(f"PASS: {len(cases)}/{len(cases)} cases agree (worst diff {worst!r})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
