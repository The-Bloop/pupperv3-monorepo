#!/usr/bin/env bash
# Keyboard teleop for pupper. Publishes to /keyboard_cmd_vel, which cmd_vel_mux treats as a
# deadband-gated manual input (press k or space -> zeros -> releases control back to autonomy).
#
# MUST run in its own interactive terminal: teleop_twist_keyboard reads stdin via termios, so
# it needs a controlling TTY. That is why it is a standalone script, not a node in launch.py.
#
# Keys: i/, = forward/back, j/l = turn left/right, k or space = stop,
#       q/z = speed up/down, w/x and e/c = tune linear/angular scale. See the on-screen legend.
#
# Environment comes from distrobox-setup/jazzy_env.sh, which ~/.bashrc sources for
# interactive shells inside pupper-jazzy. Run this from such a shell; children inherit
# it. There is no scrub here any more -- one definition, in one file.
# teleop_twist_keyboard publishes to 'cmd_vel'; remap it onto the mux's keyboard input.
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/keyboard_cmd_vel
