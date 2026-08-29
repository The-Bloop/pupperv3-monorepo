# Runbook — running Pupper V3 in MuJoCo and Isaac Sim

How to run `neural_controller` against either simulator, and how they are wired.

> **Environment.** ROS 2 for this robot is **Jazzy inside the `pupper-jazzy` distrobox**.
> The host shell is Humble and cannot run jazzy or Isaac. Every command below runs inside the
> container via `distrobox enter pupper-jazzy -- …`.
>
> **Helper scripts** live in `~/pupper/distrobox-setup/` (durable, intentionally *not* in the
> repo — they carry machine-specific paths and container-repair logic). Key ones:
> - `crun.sh` — scrub Humble from the env, source jazzy + the workspace, set CycloneDDS, then
>   exec the given ros2 command. Use it for every `ros2 …` call.
> - `launch_nc_mujoco.sh`, `launch_nc_isaac.sh` — launch the controller stack for one backend.
> - `teardown_controller.sh` — kill the controller stack (and its orphan
>   `robot_state_publisher`) while leaving the Isaac bridge alive.

## One sim at a time

MuJoCo and Isaac each start a `controller_manager` and publish `/robot_description`. Running
both at once collides on node names and topics. Tear one down before starting the other.

## MuJoCo (in-process libmujoco, headless)

```bash
distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/launch_nc_mujoco.sh
# in another shell, once controller_manager is up:
distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/crun.sh \
    ros2 control set_controller_state neural_controller active
```

No display required — MuJoCo 2.3.1 initialises headless. Selects
`pupper_v3_mujoco.urdf.xacro` → `mujoco_hardware_interface`.

## Isaac Sim — one-shot launch

Starts the Isaac bridge (Isaac's own python 3.11 + bundled jazzy rclpy) and the controller
stack together:

```bash
distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/crun.sh \
    ros2 launch pupper_isaac_sim isaac_sim.launch.py
#   optional args:  torque_mode:=model|passthrough   headless:=true|false
```

The bridge takes ~25 s to boot Isaac; the launch delays the controller start to match. Then
activate the controller as in the MuJoCo section.

## Isaac Sim — manual two-step (for debugging)

```bash
# 1. bridge first — boots Isaac (~25 s to "ROS up"), repairs the cuda stub automatically
distrobox enter pupper-jazzy -- bash \
  ~/pupper/pupperv3-monorepo/ros2_ws/src/pupper_v3_description/description/isaac_sim/run_isaac_bridge.sh \
  --headless --torque-mode model
# 2. controller stack
distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/launch_nc_isaac.sh
# 3. activate
distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/crun.sh \
    ros2 control set_controller_state neural_controller active
```

`torque_mode` on the bridge **must** match `torque_source` in
`pupper_v3_isaac.ros2_control.xacro` (`model` ↔ `isaac`, `passthrough` ↔ `plugin`).

## Verify

```bash
# prefix each with: distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/crun.sh
ros2 control list_controllers            # controller states
ros2 control list_hardware_interfaces    # isaac plugin OK == 60 command + 40 state
python3 ~/pupper/distrobox-setup/snapshot_standing.py <label>   # 12-joint pose (+ isaac body_z)
```

## Teardown before switching sim

```bash
bash ~/pupper/distrobox-setup/teardown_controller.sh
```

Kills the controller launch **and** the orphaned `robot_state_publisher` — that node reparents
to init on a plain kill and then races the next launch for `/robot_description` (which silently
loads the wrong hardware plugin). The Isaac bridge is left running; kill it by PID if you are
switching to MuJoCo for good.

## Keyboard teleop

Drive from the keyboard instead of (or alongside) the joystick. Runs in its **own terminal** —
`teleop_twist_keyboard` reads stdin via termios and needs a controlling TTY, so it is a
standalone script, not a node in `launch.py`.

```bash
# with a sim/robot + controller already up and active:
distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/keyboard_teleop.sh
```

Keys: `i`/`,` forward/back, `j`/`l` turn, `k` or space stop, `q`/`z` speed up/down.

It publishes to `/keyboard_cmd_vel`, which `cmd_vel_mux` treats as a deadband-gated manual
input at priority 1 (just below the joystick's `/teleop_cmd_vel`, above LLM and
person-follow). Press `k`/space to zero the command and hand control back to autonomy. The
input list lives in `config.yaml` under `cmd_vel_mux:`; the deadband gating for
`/keyboard_cmd_vel` is in `cmd_vel_mux_node.cpp` (rebuild `cmd_vel_mux` if you change it).

## T-Monitor — capture applied joint torque

Records the torque **actually applied** to each joint (the actuator-model output — back-EMF
and saturation already applied), for a MuJoCo-vs-Isaac comparison of the motor model itself,
not just its effect on the body. Off by default; enable with `t_monitor:=true`.

```bash
# MuJoCo, both sinks (CSV + rosbag):
distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/crun.sh \
    ros2 launch neural_controller launch.py sim:=True t_monitor:=true t_monitor_output:=both

# Isaac, CSV only:
distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/crun.sh \
    ros2 launch pupper_isaac_sim isaac_sim.launch.py t_monitor:=true t_monitor_output:=csv
```

Args (both entry points):
- `t_monitor` — `true` | `false` (default `false`). Master enable. The correct emitter is
  auto-selected by backend: **mujoco** → the C++ plugin, **isaac** → `isaac_bridge.py`.
- `t_monitor_output` — `csv` | `rosbag` | `both` (default `both`). `csv` → `torque_recorder`
  writes `results/t_monitor/<backend>_<ts>.csv`; `rosbag` → `ros2 bag record /applied_torque`
  into `results/t_monitor/<backend>_<ts>_bag`; `both` → both. The `/applied_torque` topic is
  always published when `t_monitor` is on, regardless of sink.

**Topic** `/applied_torque` (`std_msgs/Float64MultiArray`), batched physics-rate samples:
`layout.dim = [rows=N, cols=13]`, `data` = N rows of `[sim_time, tau_0 … tau_11]` in canonical
joint order. Batching keeps MuJoCo off a 10 kHz message rate (one message per 520 Hz `read()`,
N≈19 rows); Isaac flushes per `--publish-every` (default N=1 at 1 kHz).

**One run = one sim = one file.** The sims never run together (see "One sim at a time"), so a
run records a single backend. Compare by running each in turn and **diffing the two CSVs on
`sim_time`** (column 0, each sim's own clock). Do **not** row-align: the physics rates differ
(MuJoCo 10 kHz `timestep 1e-4` vs Isaac 1 kHz `--physics-dt 1e-3`, same 520 Hz command rate) —
resample on `sim_time` first. This is the same fidelity gap called out for `physics_dt`.

Captures under `results/t_monitor/` are git-ignored. Wiring: launch arg → `T_MONITOR` env on
the sim process (mujoco plugin `getenv`, bridge `os.environ`); emitters in
`mujoco_hardware_interface.cpp` / `isaac_bridge.py`; recorder `neural_controller/scripts/torque_recorder.py`.

## Gotchas

- **Host shell is Humble.** Always enter the container.
- **`crun.sh` for every ros2 call** — it scrubs Humble, sources jazzy, sets CycloneDDS.
- **Isaac cuda stub** — distrobox leaves a 0-byte `libcuda.so` that segfaults Isaac;
  `run_isaac_bridge.sh` runs `fix_isaac_cuda.sh` first, every launch.
- **Don't `pkill -f` with the process name typed into your interactive shell** — the pattern
  matches your own shell's argv and kills it (exit 144). Use the teardown script (patterns live
  in a file) or kill by PID.
- **Backlash / latency parity** — for a fair MuJoCo↔Isaac diff, MuJoCo's `backlash` is set
  `false` (Isaac's USD has none) and both carry latency `8/10`. See
  `pupper_v3_mujoco.ros2_control.xacro` and `pupper_v3_isaac.ros2_control.xacro`.
