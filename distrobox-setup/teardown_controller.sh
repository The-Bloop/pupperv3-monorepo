#!/usr/bin/env bash
# Kill the neural_controller launch + all its ros2 children, LEAVE isaac_bridge.py alone.
# Patterns live here (not in an interactive argv) so the caller's shell can't self-match.
pkill -f 'launch_nc_isaac[.]sh'      2>/dev/null
pkill -f 'ros2 launch neural_control' 2>/dev/null
pkill -f 'ros2_control_nod[e]'        2>/dev/null
pkill -f 'robot_state_publishe[r]'    2>/dev/null
pkill -f 'controller_manager.*spawne[r]' 2>/dev/null
sleep 2
echo "survivors (want only bridge):"
pgrep -af 'ros2_control_nod[e]|robot_state_publishe[r]' | grep -v teardown_controller | cat
echo "bridge procs: $(pgrep -f 'isaac_bridge[.]py' | wc -l)"
