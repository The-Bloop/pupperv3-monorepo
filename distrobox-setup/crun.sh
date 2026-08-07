#!/usr/bin/env bash
# Run a ros2 (jazzy) command inside pupper-jazzy from the host.
# Usage: bash crun.sh ros2 control list_hardware_interfaces
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
      AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH 2>/dev/null
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source /home/the_bloop/pupper/pupperv3-monorepo/ros2_ws/install_jazzy/setup.bash >/dev/null 2>&1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec "$@"
