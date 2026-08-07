#!/bin/bash
# Build the pupper ROS 2 workspace with Jazzy, inside the pupper-jazzy distrobox.
#
# Mirrors ros2_ws/build.sh at git HEAD, with one deviation: it targets build_jazzy/ +
# install_jazzy/. ros2_ws/ is bind-mounted from the host, whose Humble build owns
# build/ + install/; colcon's default bases would overwrite it.
#
#   distrobox enter pupper-jazzy -- bash ~/pupper/distrobox-setup/build_jazzy_container.sh

set -e
set -o pipefail

if [ "$CONTAINER_ID" != "pupper-jazzy" ]; then
    echo "ERROR: this must run inside the pupper-jazzy distrobox, not on the host." >&2
    echo "  distrobox enter pupper-jazzy -- bash $0" >&2
    exit 1
fi

# Scrub Humble before sourcing Jazzy. The ~/.bashrc guard does this too, but ~/.bashrc
# returns early for non-interactive shells -- so a script invoked as `bash this.sh`
# never reaches it and inherits the caller's Humble AMENT_PREFIX_PATH / PYTHONPATH /
# LD_LIBRARY_PATH straight through distrobox. Must be repeated here, not assumed.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
    AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
# sed, not `grep -v`: every LD_LIBRARY_PATH entry inherited here is a Humble path, so
# grep -v selects nothing and exits 1 -- which under `set -e -o pipefail` kills this
# script silently, before it prints anything. Empty is the correct answer, not an error.
_strip_humble() { printf '%s' "$1" | tr ':' '\n' | sed '/\/opt\/ros\/humble/d' | paste -sd: -; }
LD_LIBRARY_PATH="$(_strip_humble "$LD_LIBRARY_PATH")"
PATH="$(_strip_humble "$PATH")"
export LD_LIBRARY_PATH PATH

source /opt/ros/jazzy/setup.bash

cd "$HOME/pupper/pupperv3-monorepo/ros2_ws"

# --log-base is a colcon global option and must precede the `build` verb; after it,
# colcon exits 2 with "unrecognized arguments".
colcon --log-base log_jazzy build --symlink-install \
    --build-base build_jazzy \
    --install-base install_jazzy \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_CXX_FLAGS="-g0"

echo "=== built to ros2_ws/install_jazzy ==="
