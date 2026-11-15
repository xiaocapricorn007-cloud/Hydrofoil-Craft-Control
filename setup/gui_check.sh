#!/usr/bin/env bash
# Quick check that the full simulator starts with the Gazebo window, accepts
# operator + environment commands and keeps real time.
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh; cd /tmp
bash $S/setup/stop_sim.sh
ros2 launch hydrofoil_bringup sim.launch.py > /tmp/sim_gui.log 2>&1 &
sleep 20
HF="timeout 8 ros2 run hydrofoil_control hf"
$HF status
$HF sea 0.3 2.8 0; $HF speed 9
sleep 12
$HF status; $HF env
timeout 4 gz topic -e -t /world/foiling/stats -n 1 2>/dev/null | grep real_time_factor
echo "gui process alive: $(pgrep -fc gz-sim-gui-client)"
bash $S/setup/stop_sim.sh
echo "errors in log: $(grep -ciE 'Traceback|\[error\]|segfault' /tmp/sim_gui.log)"
grep -iE "Traceback|\[error\]|segfault" /tmp/sim_gui.log | cut -c1-200 | head -5
