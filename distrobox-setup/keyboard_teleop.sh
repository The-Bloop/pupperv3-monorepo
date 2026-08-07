#!/usr/bin/env bash
# Keyboard teleop for pupper. Publishes to /keyboard_cmd_vel, which cmd_vel_mux treats as a
# deadband-gated manual input (press k or space -> zeros -> releases control back to autonomy).
#
# MUST run in its own interactive terminal: teleop_twist_keyboard reads stdin via termios, so
# it needs a controlling TTY. That is why it is a standalone script, not a node in launch.py.
#
#   distrobox enter pupper-jazzy -- bash /home/the_bloop/pupper/distrobox-setup/keyboard_teleop.sh
#
# Keys: i/, = forward/back, j/l = turn left/right, k or space = stop,
#       q/z = speed up/down, w/x and e/c = tune linear/angular scale. See the on-screen legend.

unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
      AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH 2>/dev/null
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source /home/the_bloop/pupper/pupperv3-monorepo/ros2_ws/install_jazzy/setup.bash >/dev/null 2>&1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# teleop_twist_keyboard publishes to 'cmd_vel'; remap it onto the mux's keyboard input.
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/keyboard_cmd_vel
