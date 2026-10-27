#!/bin/bash
# Build the pupper ROS 2 workspace inside the pupper-jazzy distrobox.
#
# Thin wrapper: delegates to the workspace's own ros2_ws/build.sh. Colcon flags live there and nowhere else --
# this script used to duplicate them in order to target build_jazzy/ +
# install_jazzy/, a split that no longer exists (nothing builds with Humble).
#
#   bash ~/pupper/distrobox-setup/build_jazzy_container.sh

set -e
set -o pipefail

if [ "$CONTAINER_ID" != "pupper-jazzy" ]; then
    echo "ERROR: this must run inside the pupper-jazzy distrobox, not on the host." >&2
    echo "  distrobox enter pupper-jazzy -- bash $0" >&2
    exit 1
fi

# Environment comes from distrobox-setup/jazzy_env.sh, which ~/.bashrc sources for
# interactive shells inside pupper-jazzy. Run this from such a shell; children inherit
# it. There is no scrub here any more -- one definition, in one file.

bash "$HOME/pupper/pupperv3-monorepo/ros2_ws/build.sh"

echo "=== built to ros2_ws/install ==="
