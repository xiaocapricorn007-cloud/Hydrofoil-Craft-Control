#!/usr/bin/env bash
# Real-time factor with the full visuals (GUI + wave patch) in an irregular sea.
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh; cd /tmp
bash $S/setup/stop_sim.sh
ros2 launch hydrofoil_bringup sim.launch.py physics:=v2 > /tmp/perf.log 2>&1 &
sleep 18
ros2 run hydrofoil_control hf sea 0.4 3.0 20 spread > /dev/null 2>&1
sleep 12
echo "--- real time factor (5 samples, 2 s apart)"
for i in 1 2 3 4 5; do
  timeout 5 gz topic -e -t /world/foiling/stats -n 1 | grep -E "^real_time_factor" ; sleep 2
done
echo "--- odom rate"; timeout 12 ros2 topic hz /hydrofoil/odom --window 200 2>/dev/null | head -3
bash $S/setup/stop_sim.sh
