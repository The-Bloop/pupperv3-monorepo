#!/usr/bin/env bash
# Run a ros2 (jazzy) command inside pupper-jazzy from the host.
# Usage: bash crun.sh ros2 control list_hardware_interfaces
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
      AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH 2>/dev/null
# The host still has /opt/ros/humble and distrobox passes its env straight through.
# unset alone misses LD_LIBRARY_PATH: jazzy's setup.bash only prepends, so Humble's lib
# dirs stay on the tail and any .so present in Humble but not Jazzy loads from there.
_strip_humble() { printf '%s' "$1" | tr ':' '\n' | sed '/\/opt\/ros\/humble/d' | paste -sd: -; }
LD_LIBRARY_PATH="$(_strip_humble "${LD_LIBRARY_PATH:-}")"; export LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source /home/the_bloop/pupper/pupperv3-monorepo/ros2_ws/install/setup.bash >/dev/null 2>&1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec "$@"
