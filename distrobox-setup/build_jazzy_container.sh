#!/bin/bash
# Build the pupper ROS 2 workspace inside the pupper-jazzy distrobox.
#
# Thin wrapper: it only scrubs the host's Humble env, then delegates to the
# workspace's own ros2_ws/build.sh. Colcon flags live there and nowhere else --
# this script used to duplicate them in order to target build_jazzy/ +
# install_jazzy/, a split that no longer exists (nothing builds with Humble).
#
#   distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/build_jazzy_container.sh

set -e
set -o pipefail

if [ "$CONTAINER_ID" != "pupper-jazzy" ]; then
    echo "ERROR: this must run inside the pupper-jazzy distrobox, not on the host." >&2
    echo "  distrobox enter pupper-jazzy -- bash $0" >&2
    exit 1
fi

# Scrub Humble before build.sh sources Jazzy. The host still has /opt/ros/humble and
# distrobox passes its env straight through. The ~/.bashrc guard does this too, but
# ~/.bashrc returns early for non-interactive shells -- so a script invoked as
# `bash this.sh` never reaches it and inherits the caller's Humble AMENT_PREFIX_PATH /
# PYTHONPATH / LD_LIBRARY_PATH. Must be repeated here, not assumed.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
    AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
# sed, not `grep -v`: every LD_LIBRARY_PATH entry inherited here is a Humble path, so
# grep -v selects nothing and exits 1 -- which under `set -e -o pipefail` kills this
# script silently, before it prints anything. Empty is the correct answer, not an error.
_strip_humble() { printf '%s' "$1" | tr ':' '\n' | sed '/\/opt\/ros\/humble/d' | paste -sd: -; }
LD_LIBRARY_PATH="$(_strip_humble "$LD_LIBRARY_PATH")"
PATH="$(_strip_humble "$PATH")"
export LD_LIBRARY_PATH PATH

bash "$HOME/pupper/pupperv3-monorepo/ros2_ws/build.sh"

echo "=== built to ros2_ws/install ==="
