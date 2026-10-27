#!/usr/bin/env bash
# Rebuild only the packages touched by the latency change.
#
# Environment comes from distrobox-setup/jazzy_env.sh, which ~/.bashrc sources for
# interactive shells inside pupper-jazzy. Run this from such a shell; children inherit
# it. There is no scrub here any more -- one definition, in one file.
cd "$HOME/pupper/pupperv3-monorepo/ros2_ws" || exit 1
colcon build --symlink-install \
    --packages-select pupper_v3_description pupper_isaac_sim
