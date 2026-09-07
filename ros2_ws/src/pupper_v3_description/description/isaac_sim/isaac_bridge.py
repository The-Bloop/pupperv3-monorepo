"""Isaac Sim side of the IsaacHardwareInterface — a structural mirror of MuJoCo's.

Why this exists at all: `pupperv3_mujoco_sim` looks in-process, but it is not. Its
`read()`/`write()` call `mj_step()` ZERO times. `on_init` (mujoco_hardware_interface.cpp:143)
spawns a free-running, wall-clock-paced physics thread; the plugin only latches through two
mutex-protected globals (`latest_command_`, `d`). This module occupies exactly that seam,
with DDS standing in for the mutex:

    MuJoCo:  write() -> latest_command_ --lock--> update_ctrl_() -> model.run() -> d->ctrl -> mj_step()
    Isaac:   write() -> /isaac/joint_command --DDS--> _on_command() -> _latest --lock--> physics cb
                                                        -> model.run() -> set_joint_efforts()

so `read()`/`write()` keep the same shape: latch newest, never block, never step.

PhysX drives are deliberately NEUTRALISED (stiffness=0, damping=0, force type) and all
control goes through torque. MuJoCo does not use a PhysX-style PD — it runs a motor model
with back-EMF and saturation limits (actuator_model.py). At zero velocity that back-EMF
limit is ~1.37 Nm, BELOW the 3.0 Nm software limit, so PhysX drives would be measurably
stronger and the MuJoCo-vs-Isaac comparison would be measuring the actuator model instead
of the simulators.

Run inside the pupper-jazzy distrobox, with Isaac's BUNDLED jazzy rclpy exported first --
enabling the ros2.bridge extension does NOT put it on sys.path, and setup_ros_env.sh only
handles the C++ libs (and only when ROS_DISTRO is unset):

    export ROS_DISTRO=jazzy
    B=$HOME/isaacsim/exts/isaacsim.ros2.bridge
    export PYTHONPATH=$B/jazzy/rclpy:$PYTHONPATH
    export LD_LIBRARY_PATH=$B/jazzy/lib:$LD_LIBRARY_PATH
    ~/isaacsim/python.sh isaac_bridge.py [--torque-mode model|passthrough]

Note SimulationApp swallows stdout: prints after it may never appear even though the code
ran. Log via carb, or write to a file.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).parent.resolve()

# Wire layout, shared with IsaacHardwareInterface. Keep the two in step.
N_JOINTS = 12
CMD_LEN = 5 * N_JOINTS   # [pos_tgt x12, vel_tgt x12, ff x12, kp x12, kd x12]
STATE_LEN = 2 * N_JOINTS + 10 + 6  # [pos x12, vel x12, imu x10, body_pos x3, body_vel x3]

# sensor_msgs/JointState carries only position/velocity/effort, so it cannot express kp/kd.
# control_msgs/DynamicJointState could, but control_msgs has NO python bindings in Isaac's
# bundled rclpy (py3.11) and jazzy's are py3.12 -- unusable from here regardless of where
# Isaac runs. Float64MultiArray is bundled; the layout above is the contract.


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--torque-mode",
        choices=["model", "passthrough"],
        default="model",
        help=(
            "model: run the ported actuator model here, on FRESH per-step state (mirrors "
            "MuJoCo, which runs update_ctrl_ before every mj_step). "
            "passthrough: tau = ff, because IsaacHardwareInterface already ran the model "
            "plugin-side. Must match the plugin's torque_source param."
        ),
    )
    p.add_argument("--headless", action="store_true")
    p.add_argument("--physics-dt", type=float, default=0.0004,
                   help=("Physics step, 2.5 kHz. Safe with use_sim_time: the controller is paced by "
                         "the bridge's /clock, so control and physics stay locked no matter how far "
                         "below realtime the sim runs. Before use_sim_time, finer dt WORSENED slip "
                         "(lower realtime factor -> the wall-clock controller outran the physics); "
                         "that coupling is gone. Finer buys fidelity toward MuJoCo's 1e-4/10 kHz at "
                         "a proportional slow-motion cost (headless throughput caps ~450 steps/s "
                         "here regardless of dt). Do NOT coarsen to chase realtime -- 0.004 (250 Hz) "
                         "breaks the get-up. Report the dt gap with any MuJoCo comparison."))
    p.add_argument("--rendering-dt", type=float, default=1.0 / 60.0)
    p.add_argument("--publish-every", type=int, default=1,
                   help="Publish state every N physics steps. 1 = every step.")
    p.add_argument("--t-monitor", action="store_true",
                   help=("T-Monitor: also publish the applied joint torque on /applied_torque "
                         "at physics rate (batched every --publish-every steps). Mirrors the "
                         "MuJoCo plugin's /applied_torque so the two sims can be diffed."))
    return p.parse_args()


ARGS = parse_args()

# T-Monitor: enabled by --t-monitor OR the T_MONITOR=1 env (launch.py passes it as env, since a
# conditional CLI flag would need extra bridge variants). Either turns on /applied_torque.
T_MONITOR = ARGS.t_monitor or os.environ.get("T_MONITOR") == "1"

# SimulationApp must exist before any other isaacsim/omni/pxr import.
sys.argv.append("--/rtx/verifyDriverVersion/enabled=false")
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": ARGS.headless})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade  # noqa: E402

from isaacsim.core.api.world import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.sensors.physics import IMUSensor  # noqa: E402

from actuator_model import ActuatorParams, PIDActuatorModel  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import Float64MultiArray, MultiArrayDimension  # noqa: E402

CFG = yaml.safe_load((HERE / "config" / "pupper_v3_isaac.yaml").read_text())

# Source: pupper_v3_mujoco.ros2_control.xacro:10-14. Identical to what the MuJoCo plugin
# feeds its PIDActuatorModel, so both sims apply the same motor.
ACTUATOR_PARAMS = ActuatorParams(
    kp=0.0,   # overwritten per call by run()
    kd=0.0,
    bus_voltage=24.0,
    kt=0.04,
    phase_resistance=0.7,
    saturation_torque=4.5,
    software_torque_limit=3.0,
)

PRIM_PATH = "/World/pupper"


class IsaacBridge:
    def __init__(self) -> None:
        self.world = World(
            physics_dt=ARGS.physics_dt,
            rendering_dt=ARGS.rendering_dt,
            stage_units_in_meters=1.0,
        )
        # >>> LOAD-BEARING. Isaac's defaults here are a trampoline. <<<
        # add_default_ground_plane() defaults to static_friction=0.5, dynamic_friction=0.5,
        # restitution=0.8 (scene.py:243-245). Left at those, the robot slips (feet slide out,
        # abduction joints walk from 1.08 to 1.61 rad while the policy commands 0.98, base sinks
        # 0.173 -> 0.134, ~20 deg of yaw drift) and bounces (restitution 0.8 -- measured |lin_acc|
        # swinging 1 to 40 m/s^2 on a robot that looks stationary). Both are ground-material
        # artefacts that read as "Isaac disagrees with MuJoCo".
        #
        # Ground 0.8, and DELIBERATELY leave the feet at PhysX's default 0.5 (no foot material).
        # PhysX averages the two -> ~0.65 effective. This is the empirically-validated value the
        # robot stands and gets up on (see the ops notes / snapshot_standing history).
        #
        # Why NOT match MuJoCo's coefficient: MuJoCo's *effective* foot-ground friction is 1.0
        # (it mixes by element-wise MAX and the floor geom, pupper_v3_complete.xml:216, keeps
        # MuJoCo's default 1.0 over the foot collision class's 0.8, defaults.xml:14). But matching
        # that NUMBER in PhysX does not match the BEHAVIOUR: MuJoCo has soft/compliant contacts
        # (solimp/solref, elliptic cone) whose feet micro-slip; PhysX has rigid Coulomb contacts.
        # At the same coefficient PhysX grips harder, so 0.8/1.0 over-grip -- the front feet pin
        # during the get-up sweep and the robot faceplants instead of standing (verified: 1.0
        # jammed the front-left hip; 0.8 pinned the whole front, pitch 25 deg / roll 18 deg, never
        # stood on its own). The shared policy was validated against MuJoCo's compliant contact,
        # so the right target here is BEHAVIOURAL parity (~0.65), not coefficient parity. The
        # MuJoCo<->Isaac friction gap is intentional and documented, not a bug.
        #
        # _match_mujoco_contact() below would bind 0.8/0.8/0.0 MAX-combine to both surfaces for
        # strict coefficient parity. Left OFF for the reason above; re-enable only if a future
        # contact config (softer PhysX contacts, or a policy retrained at higher friction) can
        # tolerate the grip. restitution 0 either way (MuJoCo has none).
        self.world.scene.add_default_ground_plane(
            static_friction=0.8,
            dynamic_friction=0.8,
            restitution=0.0,
        )

        self._load_robot()
        # self._match_mujoco_contact()  # OFF: over-grips the shared policy -- see comment above.
        self._neutralise_drives()
        # The IMU prim MUST exist before reset(): PhysX registers sensors at physics
        # initialisation, and one created afterwards is never registered. It fails
        # silently -- get_current_frame() returns its default frame (identity quat, zero
        # accel) whenever imu_sensor_reading.is_valid is False (imu_sensor.py:118-121),
        # which reads exactly like a working IMU on a level robot.
        self._create_imu()

        self.world.reset()  # physics handles must exist before initialize()

        self.art = Articulation(self._articulation_root())
        self.art.initialize()
        self.imu.initialize()

        self._build_joint_index_map()
        self._init_pose()

        self.model = PIDActuatorModel(ACTUATOR_PARAMS)

        # Mirrors mujoco's latest_command_ + latest_command_lock_.
        self._lock = threading.Lock()
        self._latest: np.ndarray | None = None
        self._step_count = 0

        # T-Monitor: sim clock + per-flush batch of applied-torque rows [sim_time, tau x12].
        self._sim_time = 0.0
        self._torque_batch: list[list[float]] = []

        self._init_ros()
        self.world.add_physics_callback("bridge", self._on_physics_step)

    # ------------------------------------------------------------------ setup

    def _load_robot(self) -> None:
        usd = (HERE / CFG["asset"]["usd_path"]).resolve()
        if not usd.is_file():
            raise FileNotFoundError(f"USD not found: {usd}")
        stage = omni.usd.get_context().get_stage()
        prim = stage.DefinePrim(PRIM_PATH, "Xform")
        prim.GetReferences().AddReference(str(usd))
        simulation_app.update()  # references resolve on the next tick

        from pxr import Gf, UsdGeom

        x = UsdGeom.Xformable(stage.GetPrimAtPath(PRIM_PATH))
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(*CFG["init_state"]["pos"]))

    def _match_mujoco_contact(self) -> None:
        """Bind a physics material to the robot so foot-ground contact reproduces MuJoCo.

        Values and rationale: see the ground-plane comment in __init__. Slide friction 0.8
        (MuJoCo's deliberate foot collision-class value), restitution 0, and frictionCombineMode
        MAX to mirror MuJoCo's element-wise-max mixing. With both surfaces at 0.8 the contact
        is 0.8 under any combine mode; forcing MAX also makes the RULE match, not just the value.

        The robot USD is instanceable, so its collision shapes are instance proxies and cannot
        be edited in place. Bind on the editable /World/pupper root instead: UsdShade binding
        resolution walks a proxy's namespace ancestors, so PhysX resolves this material for
        every collision shape underneath. The material prim lives OUTSIDE the instanceable
        subtree (/World/PhysicsMaterials) so it stays editable.
        """
        stage = omni.usd.get_context().get_stage()
        material = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/pupper_contact")
        prim = material.GetPrim()
        phys = UsdPhysics.MaterialAPI.Apply(prim)
        phys.CreateStaticFrictionAttr().Set(0.8)
        phys.CreateDynamicFrictionAttr().Set(0.8)
        phys.CreateRestitutionAttr().Set(0.0)
        px = PhysxSchema.PhysxMaterialAPI.Apply(prim)
        px.CreateFrictionCombineModeAttr().Set("max")
        px.CreateRestitutionCombineModeAttr().Set("min")

        robot = stage.GetPrimAtPath(PRIM_PATH)
        binding = UsdShade.MaterialBindingAPI.Apply(robot)
        # "physics" purpose is the one PhysX reads for collision materials.
        binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
        carb.log_warn("[bridge] bound contact material 0.8/0.8/0.0 MAX-combine to robot "
                      "(mimics MuJoCo's deliberate foot-ground friction)")

    def _revolute_joints(self) -> dict[str, Usd.Prim]:
        """Traversal MUST include instance proxies -- this USD marks subtrees instanceable
        and a plain Traverse() walks straight past them, reporting zero joints."""
        stage = omni.usd.get_context().get_stage()
        pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
        return {
            p.GetName(): p
            for p in Usd.PrimRange(stage.GetPrimAtPath(PRIM_PATH), pred)
            if p.GetTypeName() == "PhysicsRevoluteJoint"
        }

    def _neutralise_drives(self) -> None:
        """Zero the PhysX PD so torque is the only input.

        The committed USD ships `drive:angular:physics:type = acceleration`, under which
        PhysX scales stiffness by joint equivalent inertia (0.000194 at the knee) -- that is
        why a naive kp=5 collapsed the robot. Irrelevant now that gains are zero, but the
        type is still forced to "force" so maxForce means Nm and any future non-zero gain
        means Nm/rad rather than an inertia-scaled fiction.
        """
        joints = self._revolute_joints()
        if not joints:
            raise RuntimeError(f"No revolute joints under {PRIM_PATH}")
        for prim in joints.values():
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(0.0)
            drive.CreateDampingAttr().Set(0.0)
            # Headroom only: the real limiting happens in the actuator model.
            drive.CreateMaxForceAttr().Set(1000.0)
        carb.log_warn(f"[bridge] neutralised {len(joints)} drives (stiffness=0, damping=0, force)")

    def _articulation_root(self) -> str:
        stage = omni.usd.get_context().get_stage()
        pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
        for p in Usd.PrimRange(stage.GetPrimAtPath(PRIM_PATH), pred):
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(p.GetPath())
        raise RuntimeError(f"No articulation root under {PRIM_PATH}")

    def _build_joint_index_map(self) -> None:
        """Map canonical order <-> Isaac DOF order.

        THE trap: Isaac orders DOFs ALPHABETICALLY (leg_back_l_1 first), while
        components.xacro, the policy observation/action vectors, and therefore the wire
        layout all use the canonical order (front_r -> front_l -> back_r -> back_l). Both
        are length 12 of the same names, so a mismatch raises nothing -- it silently
        scrambles which motor gets which command.
        """
        self.canonical = list(CFG["joint_order"])
        isaac_names = list(self.art.dof_names)
        if sorted(isaac_names) != sorted(self.canonical):
            raise RuntimeError(
                f"joint name mismatch.\n  isaac: {isaac_names}\n  canonical: {self.canonical}"
            )
        # canonical[i] lives at isaac index to_isaac[i]
        self.to_isaac = np.array([isaac_names.index(n) for n in self.canonical], dtype=np.int64)
        # isaac[i] lives at canonical index to_canon[i]
        self.to_canon = np.array([self.canonical.index(n) for n in isaac_names], dtype=np.int64)
        carb.log_warn(f"[bridge] isaac dof order: {isaac_names}")

    def _init_pose(self) -> None:
        """Snap joints to the default pose.

        Without this the articulation starts at the USD's values (all zero) and the robot
        spawns splayed on the ground.
        """
        pos_canon = np.array([CFG["default_joint_pos"][n] for n in self.canonical])
        pos_isaac = np.zeros((1, N_JOINTS), dtype=np.float32)
        pos_isaac[0, self.to_isaac] = pos_canon
        self.art.set_joint_positions(pos_isaac)
        self.default_pos_isaac = pos_isaac

    def _create_imu(self) -> None:
        """Create the IMU prim. Must run BEFORE world.reset(); initialize() after.

        Identity orientation is deliberate. components.xacro's pitch=-2.35619 is a HARDWARE
        mount correction that the real plugin applies to rotate the physical chip into
        base_link (control_board_hardware_interface.cpp:316-325). MuJoCo parses and discards
        it (mujoco_hardware_interface.cpp:83-85) because its IMU site is body-aligned.
        Isaac's IMU here is likewise already base_link-frame; applying the rotation would
        double-correct by 135 degrees.

        Note the USD's double base_link nesting: the real rigid body is <prim>/base_link/base_link.
        """
        body_root = CFG["asset"]["body_root"]
        self.imu = IMUSensor(
            prim_path=f"{PRIM_PATH}/{body_root}/base_link/imu_sensor",
            name="imu_sensor",
            translation=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # (w, x, y, z) identity
            # Moving-average widths; 1 = unfiltered.
            #
            # MEASURED: on a robot standing dead still (body z steady to 0.7 mm over 12 s)
            # this accelerometer swings |lin_acc| between 1 and 40 m/s^2 -- it should read
            # 9.81. Isaac derives acceleration by finite-differencing velocity, and at
            # dt=1e-3 that amplifies noise hugely. MuJoCo's accelerometer is analytic and
            # clean, so this is a REAL and unavoidable Isaac-vs-MuJoCo difference.
            #
            # Left unfiltered on purpose: raising these only smooths the Isaac side and adds
            # lag MuJoCo does not have, trading one divergence for a subtler one. It is
            # tolerable because the policy never reads linear_acceleration -- it uses only
            # angular_velocity and orientation (neural_controller.cpp:402-421). The accel
            # interfaces exist solely for parity with components.xacro and MuJoCo.
            # Raise these if something downstream ever does consume the accelerometer.
            linear_acceleration_filter_size=1,
            angular_velocity_filter_size=1,
            orientation_filter_size=1,
        )
        # initialize() is deliberately NOT called here -- it needs physics handles, so the
        # caller does it after world.reset().

    def _init_ros(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("isaac_bridge")
        self.node.create_subscription(
            Float64MultiArray, "/isaac/joint_command", self._on_command, 10
        )
        self.state_pub = self.node.create_publisher(Float64MultiArray, "/isaac/joint_state", 10)
        self._state_msg = Float64MultiArray()
        # /clock: this sim is the time authority for the controller stack (use_sim_time:=true on
        # the isaac backend). This CPU runs Isaac below realtime, so the wall-clock 520 Hz
        # controller would outrun the physics and the feet would slip. Driving the controller off
        # sim-time locks control and physics to ONE clock regardless of wall-clock speed -- the
        # robot just runs in slow motion, faithfully. Published every physics step from
        # self._sim_time (see _on_physics_step).
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self._clock_msg = Clock()
        # T-Monitor applied-torque publisher (only when --t-monitor). Same topic + batched
        # layout as the MuJoCo plugin.
        self.torque_pub = (
            self.node.create_publisher(Float64MultiArray, "/applied_torque", 10)
            if T_MONITOR else None
        )
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        carb.log_warn(
            f"[bridge] ROS up, torque_mode={ARGS.torque_mode}, t_monitor={T_MONITOR}, "
            f"rmw={rclpy.get_rmw_implementation_identifier()}")

    # ------------------------------------------------------------- runtime

    def _on_command(self, msg: Float64MultiArray) -> None:
        """Latch newest command. Mirrors mujoco's set_actuator_command under lock."""
        if len(msg.data) != CMD_LEN:
            carb.log_error(f"[bridge] joint_command len {len(msg.data)} != {CMD_LEN}; ignoring")
            return
        arr = np.asarray(msg.data, dtype=np.float64)
        with self._lock:
            self._latest = arr

    def _on_physics_step(self, dt: float) -> None:
        self._sim_time += dt  # sim clock, advances every physics step (T-Monitor + /clock)

        # Publish /clock first so the controller stack (use_sim_time) always has the current
        # sim-time before it acts on the state we publish below. sec/nanosec from _sim_time.
        sec = int(self._sim_time)
        self._clock_msg.clock.sec = sec
        self._clock_msg.clock.nanosec = int((self._sim_time - sec) * 1e9)
        self.clock_pub.publish(self._clock_msg)

        pos_isaac = self.art.get_joint_positions()[0]
        vel_isaac = self.art.get_joint_velocities()[0]
        pos = np.asarray(pos_isaac)[self.to_isaac]  # -> canonical
        vel = np.asarray(vel_isaac)[self.to_isaac]

        with self._lock:
            cmd = None if self._latest is None else self._latest.copy()

        if cmd is not None:
            pos_tgt = cmd[0:12]
            vel_tgt = cmd[12:24]
            ff = cmd[24:36]
            kp = cmd[36:48]
            kd = cmd[48:60]

            if ARGS.torque_mode == "passthrough":
                # The plugin already ran the model; do not re-clamp, that would apply the
                # velocity-dependent EMF limit twice against different velocities.
                tau = ff
            else:
                tau = np.array([
                    self.model.run(kp[i], kd[i], pos[i], pos_tgt[i], vel[i], vel_tgt[i], ff[i])
                    for i in range(N_JOINTS)
                ])

            tau_isaac = np.zeros((1, N_JOINTS), dtype=np.float32)
            tau_isaac[0, self.to_isaac] = tau
            self.art.set_joint_efforts(tau_isaac)

            # T-Monitor: record what we just applied (canonical order), physics rate.
            if self.torque_pub is not None:
                self._torque_batch.append([self._sim_time, *np.asarray(tau, dtype=np.float64)])

        self._step_count += 1
        if self._step_count % ARGS.publish_every == 0:
            self._publish_state(pos, vel)
            if self.torque_pub is not None and self._torque_batch:
                self._publish_torque()

    def _publish_torque(self) -> None:
        """Flush the buffered applied-torque rows as one batched message. Layout mirrors the
        MuJoCo plugin: N rows x (1 + N_JOINTS) cols, each [sim_time, tau x12] canonical."""
        rows = self._torque_batch
        n = len(rows)
        cols = 1 + N_JOINTS
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="rows", size=n, stride=n * cols),
            MultiArrayDimension(label="cols", size=cols, stride=cols),
        ]
        flat: list[float] = []
        for r in rows:
            flat.extend(r)
        msg.data = flat
        self.torque_pub.publish(msg)
        self._torque_batch = []

    def _publish_state(self, pos: np.ndarray, vel: np.ndarray) -> None:
        """One atomic snapshot, mirroring mujoco's single locked read of `d`.

        Two topics would tear: the plugin could pair joint state from one instant with IMU
        from another. MuJoCo reads both out of the same `d` under one lock, so this is one
        message.
        """
        frame = self.imu.get_current_frame(read_gravity=True)
        q = np.asarray(frame["orientation"])          # (w, x, y, z)
        ang = np.asarray(frame["ang_vel"])
        lin = np.asarray(frame["lin_acc"])

        # Ground truth, mirroring mujoco_sensor/body_pos|body_vel
        # (mujoco_hardware_interface.cpp:186-197). Nothing consumes it -- it is what the
        # MuJoCo-vs-Isaac comparison actually diffs.
        body_pos, _ = self.art.get_world_poses()
        body_vel = self.art.get_linear_velocities()

        out = np.empty(STATE_LEN, dtype=np.float64)
        out[0:12] = pos
        out[12:24] = vel
        # ros2_control imu_sensor order: orientation x,y,z,w then ang_vel then lin_acc.
        # Isaac gives (w,x,y,z); reorder rather than assume.
        out[24:28] = [q[1], q[2], q[3], q[0]]
        out[28:31] = ang
        out[31:34] = lin
        out[34:37] = np.asarray(body_pos)[0]
        out[37:40] = np.asarray(body_vel)[0]

        self._state_msg.data = out.tolist()
        self.state_pub.publish(self._state_msg)

    # Max wall-clock drift before we re-sync instead of trying to catch up. Analog of MuJoCo's
    # syncmisalign (mujoco_core_interactive.hpp:44). Sprinting to catch up a stall would spike
    # the command-to-contact ratio and make the feet skid -- the very thing we are fixing.
    PACE_RESYNC_S = 0.05

    def run(self) -> None:
        # Wall-clock pace the physics, mirroring MuJoCo's sync loop
        # (mujoco_core_interactive.hpp:1893). The controller_manager runs on wall-clock at
        # 520 Hz (no use_sim_time, no /clock published), so the sim-clock MUST track real time
        # or the gait desyncs and the feet slip. A free-running world.step() was the root cause
        # of Isaac slipping more than MuJoCo, and of finer physics_dt making it worse (more
        # compute per sim-second -> further behind real time). If the machine can't hold real
        # time we fall behind and re-sync rather than sprint.
        dt = ARGS.physics_dt
        sim_elapsed = 0.0
        wall_start = time.monotonic()
        try:
            while simulation_app.is_running():
                self.world.step(render=not ARGS.headless)
                sim_elapsed += dt
                lead = sim_elapsed - (time.monotonic() - wall_start)
                if lead > 0:
                    time.sleep(lead)                      # ahead of real time -> wait
                elif lead < -self.PACE_RESYNC_S:          # too far behind -> re-sync, don't sprint
                    wall_start = time.monotonic() - sim_elapsed
        finally:
            rclpy.shutdown()
            simulation_app.close()


if __name__ == "__main__":
    IsaacBridge().run()
