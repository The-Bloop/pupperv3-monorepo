#!/usr/bin/env bash
# Launch neural_controller against the in-process MuJoCo sim. Runs INSIDE pupper-jazzy.
# No `set -u`: jazzy setup.bash references AMENT_TRACE_SETUP_FILES unbound.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
      AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH 2>/dev/null
_strip_humble() { printf '%s' "$1" | tr ':' '\n' | sed '/\/opt\/ros\/humble/d' | paste -sd: -; }
LD_LIBRARY_PATH="$(_strip_humble "${LD_LIBRARY_PATH:-}")"; export LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source /home/the_bloop/pupper/pupperv3-monorepo/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

exec ros2 launch neural_controller launch.py \
    backend:=mujoco \
    teleop:=False \
    bag_recorder:=False
