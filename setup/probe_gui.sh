#!/usr/bin/env bash
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh; cd /tmp
bash $S/setup/stop_sim.sh
ros2 launch hydrofoil_bringup sim.launch.py > /tmp/gui_probe.log 2>&1 &
sleep 16
for mode in FOLLOW; do
  timeout 5 gz topic -t /gui/track -m gz.msgs.CameraTrack \
     -p "track_mode: $mode, follow_target: {name: \"hydrofoil_craft\"}, follow_offset: {x: -11, y: 6, z: 4}, follow_pgain: 1.0"
  sleep 4
  echo "--- mode $mode: currently tracked"; timeout 4 gz topic -e -t /gui/currently_tracked -n 1
  for i in 1 2; do
    echo "--- camera pose / craft status"; timeout 4 gz topic -e -t /gui/camera/pose -n 1 | tr '\n' ' '; echo
    timeout 5 gz topic -e -t /hydrofoil/odom -n 1 | grep -A3 "position" | tr '\n' ' '; echo
    sleep 3
  done
done
bash $S/setup/stop_sim.sh
