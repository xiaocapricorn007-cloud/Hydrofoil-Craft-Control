#!/usr/bin/env bash
# Is the GUI simulation advancing?  sim time, RTF, craft position, camera.
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh; cd /tmp
bash $S/setup/stop_sim.sh
ros2 launch hydrofoil_bringup sim.launch.py > /tmp/diag.log 2>&1 &
sleep 22
for i in 1 2 3; do
  echo "--- sample $i"
  timeout 5 gz topic -e -t /world/foiling/stats -n 1 | grep -E "^real_time_factor|sec:" | head -3 | tr '\n' ' '; echo
  timeout 5 gz topic -e -t /hydrofoil/odom -n 1 | grep -A2 "position" | tr '\n' ' '; echo
  timeout 5 gz topic -e -t /gui/camera/pose -n 1 | grep -A2 "position" | tr '\n' ' '; echo
  sleep 3
done
echo "--- gz processes"; ps aux | grep -E "gz sim" | grep -v grep | awk '{print $3, $4, $11, $12, $13, $14}'
bash $S/setup/stop_sim.sh
