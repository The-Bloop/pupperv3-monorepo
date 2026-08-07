"""Python port of pupperv3_mujoco_sim's PIDActuatorModel.

Line-for-line port of ros2_ws/src/pupperv3_mujoco_sim/lib/actuator_model.hpp:66-87, so that
Isaac and MuJoCo apply the *same* actuator model and a comparison between them measures the
simulators rather than the motor model.

This model is why Isaac's PhysX drives are not used: PhysX offers only tau = kp*err +
kd*derr clamped by maxForce, with no back-EMF or saturation limit. At zero velocity the
back-EMF limit here evaluates to ~1.37 Nm, which *binds below* the 3.0 Nm software limit --
so PhysX drives would be visibly stronger than MuJoCo's actuators.

Parity with the C++ is enforced by test_actuator_model_parity.py. Keep the two in step: any
edit here without the same edit there silently invalidates every comparison.
"""

from __future__ import annotations

from dataclasses import dataclass


def _clamp(v: float, lo: float, hi: float) -> float:
    """Mirror libstdc++'s std::clamp exactly: `v < lo ? lo : hi < v ? hi : v`.

    Deliberately NOT min(max(v, lo), hi). The two agree only while lo <= hi, and the C++
    can be called with lo > hi: emf_limited_torque goes negative once |kt*velocity| exceeds
    bus_voltage (|velocity| > 600 rad/s at kt=0.04, bus_voltage=24), which makes
    std::clamp(v, -x, x) undefined behaviour with x < 0. libstdc++ returns lo there; the
    min/max spelling returns hi. Only reachable in a diverging sim -- i.e. exactly when a
    MuJoCo-vs-Isaac diff is being read -- so match the real behaviour rather than fix it.
    """
    if v < lo:
        return lo
    if hi < v:
        return hi
    return v


@dataclass
class ActuatorParams:
    """Mirrors the C++ struct (actuator_model.hpp:14-45).

    Values come from pupper_v3_mujoco.ros2_control.xacro:10-14. The C++ ctor asserts every
    field is >= 0; __post_init__ does the same so a bad config fails here rather than
    producing quietly wrong torques.
    """

    kp: float                     # Nm/rad
    kd: float                     # Nm/(rad/s)
    bus_voltage: float            # V
    kt: float                     # Nm/A (careful about winding type)
    phase_resistance: float       # Ohms (careful about winding type)
    saturation_torque: float      # Nm
    software_torque_limit: float  # Nm

    def __post_init__(self) -> None:
        for name in (
            "kp", "kd", "bus_voltage", "kt",
            "phase_resistance", "saturation_torque", "software_torque_limit",
        ):
            if getattr(self, name) < 0:
                raise ValueError(f"ActuatorParams.{name} must be >= 0, got {getattr(self, name)}")


class PIDActuatorModel:
    """Port of PIDActuatorModel (actuator_model.hpp:47-99)."""

    def __init__(self, params: ActuatorParams) -> None:
        self.params = params

    def run(
        self,
        kp: float,                 # Nm/rad
        kd: float,                 # Nm/(rad/s)
        position: float,           # rad
        position_target: float,    # rad
        velocity: float,           # rad/s
        velocity_target: float,    # rad/s
        feedforward_torque: float,  # Nm
    ) -> float:
        """Port of the 7-arg run() overload, which sets kp/kd then runs the 5-arg one.

        The C++ mutates params_.kp/kd on every call (actuator_model.hpp:62-63) -- kept here
        so behaviour matches if anything ever reads the params back.
        """
        self.params.kp = kp
        self.params.kd = kd

        p = self.params
        torque_command = (
            p.kp * (position_target - position)
            + p.kd * (velocity_target - velocity)
            + feedforward_torque
        )
        torque_command = _clamp(torque_command, -p.software_torque_limit, p.software_torque_limit)
        emf = abs(p.kt * velocity)
        max_current = (p.bus_voltage - emf) / p.phase_resistance
        emf_limited_torque = max_current * p.kt
        torque_command = _clamp(torque_command, -emf_limited_torque, emf_limited_torque)
        torque_command = _clamp(torque_command, -p.saturation_torque, p.saturation_torque)
        return torque_command
