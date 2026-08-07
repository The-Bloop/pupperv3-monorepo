#!/bin/bash
# Launch isaac_bridge.py with Isaac's BUNDLED jazzy rclpy on the path.
#
# Isaac uses its own python 3.11 and ships a matching jazzy rclpy; jazzy's own rclpy is
# python 3.12 and cannot be imported here. Neither enabling the isaacsim.ros2.bridge
# extension nor sourcing setup_ros_env.sh puts the bundled one on sys.path --
# setup_ros_env.sh only sets LD_LIBRARY_PATH for the C++ libs, and only when ROS_DISTRO is
# unset (so it no-ops in any shell that already sourced ROS). Hence this wrapper.
#
# Run inside the pupper-jazzy distrobox:
#   distrobox enter pupper-jazzy -- bash .../run_isaac_bridge.sh [--headless] [--torque-mode model|passthrough]

set -e
set -o pipefail

if [ "$CONTAINER_ID" != "pupper-jazzy" ]; then
    echo "ERROR: run inside the pupper-jazzy distrobox." >&2
    echo "  distrobox enter pupper-jazzy -- bash $0 $*" >&2
    exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC="$HOME/isaacsim"
BRIDGE="$ISAAC/exts/isaacsim.ros2.bridge"

# distrobox --nvidia intermittently leaves libcuda.so as a 0-byte stub, which makes Isaac
# segfault inside SimulationApp.__init__ with a traceback that points at this script rather
# than at the real cause. It recurs across restarts, so check every launch.
CUDA_FIX="$HOME/pupper/distrobox-setup/fix_isaac_cuda.sh"
if [ -x "$CUDA_FIX" ] || [ -f "$CUDA_FIX" ]; then
    bash "$CUDA_FIX"
else
    echo "WARNING: $CUDA_FIX not found; skipping the libcuda.so stub check." >&2
    echo "         If Isaac segfaults during startup, that is almost certainly why." >&2
fi

if [ ! -d "$BRIDGE/jazzy/rclpy" ]; then
    echo "ERROR: bundled jazzy rclpy not found at $BRIDGE/jazzy/rclpy" >&2
    exit 1
fi

# Isaac's bundled ROS2 only. Do NOT source /opt/ros/jazzy here: that would put jazzy's
# python 3.12 site-packages ahead of the 3.11 bundle and rclpy would fail to import.
export ROS_DISTRO=jazzy
export PYTHONPATH="$BRIDGE/jazzy/rclpy:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$BRIDGE/jazzy/lib:${LD_LIBRARY_PATH:-}"
# Match ~/.bashrc, which pins cyclone for the whole box. The bundle ships cyclone and
# fastrtps; Isaac's own default is fastrtps, so this must be explicit or the bridge and
# ros2_control land on different RMWs and silently never see each other.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

cd "$HERE"
exec "$ISAAC/python.sh" "$HERE/isaac_bridge.py" "$@"
