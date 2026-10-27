#!/usr/bin/env bash
# Run a command in the already-prepared Jazzy environment.
#   bash crun.sh ros2 control list_hardware_interfaces
#
# It used to scrub Humble and source Jazzy + the workspace itself. That moved to
# distrobox-setup/jazzy_env.sh. Kept as a passthrough because lab_5_fall_2025's eval
# backend invokes it by path (eval.ros_crun), and because it marks the places that
# expect a prepared shell.
#
# Environment comes from distrobox-setup/jazzy_env.sh, which ~/.bashrc sources for
# interactive shells inside pupper-jazzy. Run this from such a shell; children inherit
# it. There is no scrub here any more -- one definition, in one file.
exec "$@"
