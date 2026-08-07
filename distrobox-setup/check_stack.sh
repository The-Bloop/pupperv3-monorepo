#!/usr/bin/env bash
# Report what's alive: bridge process, orphan ros2_control/rsp, and CM responsiveness.
b=$(pgrep -af 'isaac_bridge[.]py' | grep -c python)
echo "bridge python procs: $b"
echo "--- orphan control nodes (want 0 after a kill) ---"
pgrep -af 'ros2_control_nod[e]|robot_state_publishe[r]' | grep -v distrobox-setup | cat
echo "--- (end) ---"
