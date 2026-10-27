# Jazzy environment for the pupper-jazzy distrobox. SOURCED, never executed --
# it mutates the calling shell.
#
# ~/.bashrc sources this for interactive shells whose CONTAINER_ID is pupper-jazzy,
# so an interactive container shell is ready without sourcing anything by hand.
# It lives in the repo because the whole workspace depends on it: every ros2 call,
# every launch and the lab_5 eval backend assume this ran.
#
# Non-interactive shells never read ~/.bashrc, so `bash script.sh` and
# `distrobox enter -- cmd` do NOT get this. Children of a prepared interactive
# shell inherit it, which is how subprocesses in eval.py see the workspace.

# distrobox hands the container the host shell's entire environment, and on the host
# Humble is sourced -- so AMENT_PREFIX_PATH, PYTHONPATH and LD_LIBRARY_PATH arrive
# already carrying /opt/ros/humble. Sourcing Jazzy on top of that is what its "do not
# mix paths from different distributions" warning is about: Humble's python3.10
# site-packages and its 22.04 .so files would sit in this python3.12 container's
# search path. Scrub Humble out before sourcing Jazzy.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY \
    AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH

# sed, not `grep -v`: every inherited LD_LIBRARY_PATH entry is a Humble path, so
# grep -v selects nothing and exits 1 -- fatal under `set -e`.
_strip_humble() { printf '%s' "$1" | tr ':' '\n' | sed '/\/opt\/ros\/humble/d' | paste -sd: -; }
LD_LIBRARY_PATH="$(_strip_humble "${LD_LIBRARY_PATH:-}")"
PATH="$(_strip_humble "$PATH")"
export LD_LIBRARY_PATH PATH
unset -f _strip_humble

# set by the host branch of ~/.bashrc (turtlebot3), and the only /opt/ros/humble
# path that otherwise survives into a container shell. Nothing here uses Gazebo.
unset GAZEBO_MODEL_PATH

source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/jazzy/lib/

_pupper_ws=$HOME/pupper/pupperv3-monorepo/ros2_ws/install/setup.bash
if [ -f "$_pupper_ws" ]; then
    source "$_pupper_ws"
else
    echo "jazzy_env.sh: workspace not built, $_pupper_ws missing -- run ros2_ws/build.sh" >&2
fi
unset _pupper_ws

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_COLORIZED_OUTPUT=1
