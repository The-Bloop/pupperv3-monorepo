#!/usr/bin/env bash
# Launch a sim WITH joystick teleop enabled, and capture per-process logs.
# Runs INSIDE pupper-jazzy. Host Bash tool must invoke via:
#   distrobox enter pupper-jazzy -- bash /home/the_bloop/pupper/distrobox-setup/launch_nc_teleop.sh [backend] [headless]
#     backend  = mujoco (default) | isaac | real
#     headless = false (default, isaac only) | true    -- ignored for mujoco/real
#
# All three backends go through neural_controller/launch.py now: it starts isaac_bridge.py
#   itself when backend:=isaac (holding the controller stack back by startup_delay), and the
#   sim is in-process for mujoco/real. `headless` is isaac-only (GUI by default so you can
#   watch while driving); it is ignored for mujoco/real.
# No `set -u`: jazzy's setup.bash references AMENT_TRACE_SETUP_FILES unbound.

BACKEND="${1:-mujoco}"
HEADLESS="${2:-false}"

# scrub any humble leakage (non-interactive shells skip the .bashrc guard)
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
      AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH 2>/dev/null
_strip_humble() { printf '%s' "$1" | tr ':' '\n' | sed '/\/opt\/ros\/humble/d' | paste -sd: -; }
LD_LIBRARY_PATH="$(_strip_humble "${LD_LIBRARY_PATH:-}")"; export LD_LIBRARY_PATH

source /opt/ros/jazzy/setup.bash
source /home/the_bloop/pupper/pupperv3-monorepo/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- logging -------------------------------------------------------------
# Force every node's rcl log into ONE run-specific dir so each process gets
# its own <node>_<pid>_<n>.log there (ros2 launch also drops launch.log + a
# per-node stdout capture). Predictable path instead of ~/.ros/log/<random>.
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$HOME/pupper/distrobox-setup/results/teleop_logs/${BACKEND}_${STAMP}"
mkdir -p "$RUN_DIR"
export ROS_LOG_DIR="$RUN_DIR"

COMBINED="$RUN_DIR/combined.log"
echo "== teleop launch: backend=$BACKEND  $(date) =="        | tee "$COMBINED"
echo "   per-process logs (rcl) : $RUN_DIR/<node>_<pid>_*.log" | tee -a "$COMBINED"
echo "   combined stdout/stderr : $COMBINED"                   | tee -a "$COMBINED"
echo "   joystick device        : /dev/input/js0 -> $(ls /dev/input/js0 2>/dev/null || echo MISSING)" | tee -a "$COMBINED"
echo "==========================================================" | tee -a "$COMBINED"

# tee the whole launch (all nodes run output="both") to the combined log while
# still showing it live. Not `exec` here: a pipeline can't be exec'd, so the
# script shell stays as the pipeline's parent and waits. Ctrl-C / killing this
# script's PID takes the whole launch down with it.
ros2 launch neural_controller launch.py \
    backend:="$BACKEND" \
    teleop:=True \
    headless:="$HEADLESS" \
    bag_recorder:=False \
  2>&1 | tee -a "$COMBINED"
