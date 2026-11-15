#!/usr/bin/env bash
# Launch the full simulator with the GUI and check that the camera follows the
# craft by itself (no manual /gui/track request).
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh; cd /tmp
bash $S/setup/stop_sim.sh
ros2 launch hydrofoil_bringup sim.launch.py > /tmp/follow_check.log 2>&1 &
sleep 24
echo "--- currently tracked"; timeout 5 gz topic -e -t /gui/currently_tracked -n 1 | tr '\n' ' '; echo
ros2 run hydrofoil_control hf speed 10 > /dev/null 2>&1
for i in 1 2 3; do
  sleep 4
  echo "--- t+$((i*4))s camera pose / craft position"
  timeout 4 gz topic -e -t /gui/camera/pose -n 1 | grep -A3 "position" | tr '\n' ' '; echo
  timeout 5 gz topic -e -t /hydrofoil/odom -n 1 | grep -A3 "position" | tr '\n' ' '; echo
done
bash $S/setup/stop_sim.sh
