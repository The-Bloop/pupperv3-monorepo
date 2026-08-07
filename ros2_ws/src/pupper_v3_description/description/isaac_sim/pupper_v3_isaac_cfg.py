"""Isaac Lab configuration for the Pupper V3 quadruped.

The numbers live in config/pupper_v3_isaac.yaml -- this module only translates them into
Isaac Lab cfg objects. Edit the YAML, not this file.

IMPORTANT: import this module only AFTER isaaclab.app.AppLauncher has started. isaaclab
pulls in `pxr`, which isaacsim injects onto sys.path at app launch; importing earlier
fails with "ImportError: from pxr import Usd".

Run with the Isaac Lab interpreter (NOT ~/isaacsim/python.sh, which has no isaaclab):
    /home/the_bloop/isaac-lab/IsaacLab/isaaclab.sh -p your_script.py

Typical use:

    from isaaclab.app import AppLauncher
    app_launcher = AppLauncher(headless=True)

    from pupper_v3_isaac_cfg import PUPPER_V3_CFG, JOINT_ORDER, make_imu_cfg

    robot_cfg = PUPPER_V3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    imu_cfg = make_imu_cfg("{ENV_REGEX_NS}/Robot")
"""

import warnings
from pathlib import Path

import yaml

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sensors import ContactSensorCfg, ImuCfg

_HERE = Path(__file__).parent.resolve()
_CONFIG_PATH = _HERE / "config" / "pupper_v3_isaac.yaml"


def load_config(path: Path = _CONFIG_PATH) -> dict:
    """Read the YAML config.

    Raises FileNotFoundError if absent and yaml.YAMLError if malformed. Both are better
    errors than anything this function could substitute, so neither is caught.
    """
    return yaml.safe_load(path.read_text())


def _usd_path(cfg: dict) -> str:
    """Resolve the USD path relative to this directory."""
    path = _HERE / cfg["asset"]["usd_path"]
    if not path.exists():
        # Isaac's own error for a missing USD is opaque, so name the path here. This warns
        # and proceeds -- it does not substitute a different asset.
        warnings.warn(f"USD asset not found at {path}; Isaac will fail to spawn the robot.")
    return str(path)


def make_articulation_cfg(cfg: dict | None = None) -> ArticulationCfg:
    """Build the Pupper V3 articulation cfg.

    The returned cfg has no prim_path; set it with .replace(prim_path=...) at the call
    site, matching the isaaclab_assets convention.
    """
    if cfg is None:
        cfg = load_config()

    asset = cfg["asset"]
    init_state = cfg["init_state"]
    actuator = cfg["actuator"]

    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=_usd_path(cfg),
            activate_contact_sensors=asset["activate_contact_sensors"],
            # Load-bearing: the USD ships acceleration drives, which make the gains below
            # ~5000x too weak at the knee and collapse the robot. See the drive_type note
            # in the YAML.
            joint_drive_props=sim_utils.JointDrivePropertiesCfg(
                drive_type=asset["drive_type"],
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=asset["enabled_self_collisions"],
                solver_position_iteration_count=asset["solver_position_iteration_count"],
                solver_velocity_iteration_count=asset["solver_velocity_iteration_count"],
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(init_state["pos"]),
            rot=tuple(init_state["rot"]),
            joint_pos=dict(cfg["default_joint_pos"]),
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=cfg["soft_joint_pos_limit_factor"],
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=actuator["joint_names_expr"],
                # The *_sim names are required: ImplicitActuatorCfg deprecates plain
                # `effort_limit` and silently ignores plain `velocity_limit`.
                effort_limit_sim=actuator["effort_limit_sim"],
                velocity_limit_sim=actuator["velocity_limit_sim"],
                stiffness=actuator["stiffness"],
                damping=actuator["damping"],
                armature=actuator["armature"],
                friction=actuator["friction"],
            )
        },
    )


def make_imu_cfg(prim_path: str, cfg: dict | None = None) -> ImuCfg:
    """Build the base_link IMU cfg.

    prim_path is the robot's prim path, e.g. "{ENV_REGEX_NS}/Robot". The body_root
    wrapper level and the IMU body are appended to it -- see the double base_link nesting
    note in the YAML.

    The offset is identity on purpose. See the IMU trap comment in the YAML: the
    pitch=-2.35619 in components.xacro is a hardware mount correction, and applying it
    here would double-correct.
    """
    if cfg is None:
        cfg = load_config()

    body_root = cfg["asset"]["body_root"]
    imu = cfg["sensors"]["imu"]
    return ImuCfg(
        prim_path=f"{prim_path}/{body_root}/{imu['prim_body']}",
        offset=ImuCfg.OffsetCfg(pos=tuple(imu["offset_pos"]), rot=tuple(imu["offset_rot"])),
        gravity_bias=tuple(imu["gravity_bias"]),
        update_period=imu["update_period"],
    )


def make_contact_sensor_cfg(prim_path: str, cfg: dict | None = None) -> ContactSensorCfg:
    """Build the foot contact sensor cfg covering all four leg_*_3 links.

    prim_path is the robot's prim path; the body_root wrapper level is inserted -- see the
    double base_link nesting note in the YAML.

    Reads zero forever unless the articulation was spawned with
    activate_contact_sensors=True, which make_articulation_cfg does.
    """
    if cfg is None:
        cfg = load_config()

    body_root = cfg["asset"]["body_root"]
    contact = cfg["sensors"]["contact"]
    return ContactSensorCfg(
        prim_path=f"{prim_path}/{body_root}/{contact['body_names_expr'][0]}",
        update_period=contact["update_period"],
        history_length=contact["history_length"],
        track_air_time=contact["track_air_time"],
        force_threshold=contact["force_threshold"],
    )


_CFG = load_config()

PUPPER_V3_CFG: ArticulationCfg = make_articulation_cfg(_CFG)
"""Pupper V3 articulation cfg. Set prim_path via .replace() at the call site."""

JOINT_ORDER: list[str] = _CFG["joint_order"]
"""Canonical policy joint order. Isaac reports joints ALPHABETICALLY, which is a different
order -- reindex through this list before feeding a trained policy."""
