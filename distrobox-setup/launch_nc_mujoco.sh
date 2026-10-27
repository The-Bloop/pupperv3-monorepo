#!/usr/bin/env bash
# Launch neural_controller against the in-process MuJoCo sim.
#
# Environment comes from distrobox-setup/jazzy_env.sh, which ~/.bashrc sources for
# interactive shells inside pupper-jazzy. Run this from such a shell; children inherit
# it. There is no scrub here any more -- one definition, in one file.
exec ros2 launch neural_controller launch.py \
    backend:=mujoco \
    teleop:=False \
    bag_recorder:=False
