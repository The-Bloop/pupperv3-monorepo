# Isaac Sim assets and config for Pupper V3

Everything Isaac Sim needs for the Pupper V3, split out from `../urdf/` (which holds the
MuJoCo-lineage URDFs and the `.edited.*` variants used by the ROS2 stack).

```
isaac_sim/
├── pupper_v3_isaac.urdf          copy of ../urdf/pupper_v3_isaac.urdf
├── pupper_v3_isaac/              the committed USD asset (GUI-authored)
│   ├── pupper_v3_isaac.usd       root stage; subLayers into configuration/
│   └── configuration/            base (geometry) / physics / robot / sensor layers
├── config/pupper_v3_isaac.yaml   all the numbers -- single source of truth
├── pupper_v3_isaac_cfg.py        loader: YAML -> ArticulationCfg / ImuCfg / ContactSensorCfg
└── convert_urdf_to_usd.py        URDF -> USD, writes to pupper_v3_isaac_generated/
```

The URDF sits at the folder root, not in a `urdf/` subdir, so its `../meshes/stl/*.stl`
references resolve at the same depth as the original. Moving it down a level would break
all 13 mesh paths.

## Two interpreters

This folder needs **two different Python interpreters**, and mixing them up produces
confusing import errors:

| File | Interpreter | Why |
|---|---|---|
| `pupper_v3_isaac_cfg.py` | `/home/the_bloop/isaac-lab/IsaacLab/isaaclab.sh -p` | needs `isaaclab` |
| `convert_urdf_to_usd.py` | `~/isaacsim/python.sh` | needs only `isaacsim` |

`~/isaacsim/python.sh` does **not** have `isaaclab` installed — the config module cannot
run under it.

`pupper_v3_isaac_cfg.py` must be imported **after** `AppLauncher` has started. `isaaclab`
pulls in `pxr`, which isaacsim injects onto `sys.path` at app launch; importing earlier
fails with `ImportError: from pxr import Usd`.

```python
from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)      # must come first

from pupper_v3_isaac_cfg import PUPPER_V3_CFG, JOINT_ORDER, make_imu_cfg
robot_cfg = PUPPER_V3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
```

## Verified working

Spawned in Isaac Lab at the config's gains, the robot stands:

| Check | Result |
|---|---|
| Total mass | 2.9000 kg (1.5 base + 1.4 legs) |
| Joints / bodies | 12 revolute / 13 |
| Joint limits in USD | match the URDF exactly |
| Feet coplanar at default pose | spread **0.00000** |
| Standing height (measured) | base z = **0.16139** |
| Settled height under gravity | **0.1582** (~0.3 cm PD droop) |
| Max joint error under load | **0.0735 rad** (4.2°) |
| Max torque under load | **0.321 Nm** of the 3.0 Nm limit |
| Foot contact total | **28.456 N** vs 28.449 N expected (m·g) — evenly across all four |
| IMU | `lin_acc_b` ≈ +z, `ang_vel_b` ≈ 0, `quat_w` ≈ identity |

## THE DRIVE-TYPE TRAP (this one cost a day)

**The committed USD ships `drive:angular:physics:type = acceleration` on all 12 joints.**

Under an acceleration drive PhysX scales stiffness by the joint's equivalent inertia,
which here is tiny — `leg_*_1` 0.00726, `leg_*_2` 0.00114, **`leg_*_3` (knee) 0.000194**.
So a stiffness of 5.0 produces roughly `5 × err × 0.000194 ≈ 0.001 Nm` at the knee: about
5000× too weak.

The symptom is deeply misleading. The robot sags until all four knees reach their joint
limits and stop there, with joint error exactly equal to the target-to-limit distance
(1.2300 rad). It looks like an inverted joint axis or a broken asset. It is not — and
**raising the gains does not help**: kp=200 with a 1000 Nm effort limit collapses exactly
the same way, because the inertia scaling defeats any kp you choose.

`ImplicitActuatorCfg` writes stiffness/damping but does **not** touch the drive type, so it
has to be forced at spawn. `pupper_v3_isaac_cfg.py` does this via
`UsdFileCfg(joint_drive_props=JointDrivePropertiesCfg(drive_type="force"))`, driven by
`asset.drive_type` in the YAML. Remove it and the robot collapses again.

If you drive this USD by hand (raw `pxr`, no Isaac Lab), set
`UsdPhysics.DriveAPI.Apply(prim, "angular").CreateTypeAttr().Set("force")` yourself — and
remember USD angular drives are in **degrees**, so kp/kd need a `π/180` conversion and
targets need `math.degrees()`.

## Three traps

**1. IMU double-correction.** `../components.xacro` declares the IMU with
`pitch=-2.35619` (-3π/4). That is a *mount-orientation correction* the ros2_control
hardware interface applies to rotate the physical chip's raw readings into the
`base_link` frame — not a pose, and there is no sensor link. Isaac's IMU on `base_link`
with an identity offset already gives base_link-frame data. Applying that rotation here
would correct twice and silently rotate the observation by 135°. `offset_rot` is identity
on purpose.

**2. Joint order.** Isaac reports joints **alphabetically** (`leg_back_l_1` first). The
policy's canonical order is `front_r → front_l → back_r → back_l`. The observation and
action vectors are index-based, so a mismatch scrambles the robot without raising.
Reindex through `JOINT_ORDER`.

**3. Gains differ between sim and hardware.** The YAML defaults to the training values
(kp 5.0, kd 0.25, from `ai/rl/conf/training/default.yaml`). The deployed robot runs
kp 7.5 (`neural_controller/launch/config.yaml`). That 1.5x gap sits outside the
`[0.6, 1.1]` kp randomization range used in training — documented, not fixed. Set
`stiffness: 7.5` to reproduce the hardware feel.

## Regenerating the USD

`convert_urdf_to_usd.py` writes to `pupper_v3_isaac_generated/`, **not** over
`pupper_v3_isaac/`. The committed asset was authored in the Isaac Sim GUI and carries a
variantSet structure (Physics/Sensor/Robot with payloads) that the script's flat import
does not reproduce — overwriting would silently downgrade it. The generated output is a
cross-check, and is gitignored.

`pupper_v3_isaac_sensor.usd` is an empty stub with no sensor prims. That is fine: Isaac
Lab's `ImuCfg` and `ContactSensorCfg` attach to existing rigid-body prims and spawn their
sensors at runtime, so no USD sensor prims are needed. The Sensor variant exists for
GUI/ROS workflows.

## Inspecting the USD

Counting colliders with a plain `stage.Traverse()` returns **0** and is misleading — the
`collisions` Xforms are `instanceable=True`, and default traversal skips instance proxies.
Use `Usd.TraverseInstanceProxies`; the correct count is 9 (1 body box, 4 leg cylinders,
4 foot spheres).
