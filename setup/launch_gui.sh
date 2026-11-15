#!/usr/bin/env bash
# Start the full simulator with the Gazebo window (stops any old run first).
#   bash launch_gui.sh [physics]      physics: ref (default) | v2 | switch names
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh; cd /tmp
bash $S/setup/stop_sim.sh
exec ros2 launch hydrofoil_bringup sim.launch.py physics:=${1:-ref} > /tmp/sim_gui.log 2>&1
