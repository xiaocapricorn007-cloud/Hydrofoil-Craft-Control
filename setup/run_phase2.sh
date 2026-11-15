#!/usr/bin/env bash
# Scripted acceptance run in Gazebo (headless): operator + environment actions
# from a scenario file, trajectory logged to logs/gz_phase2_<name>.npz
#   run_phase2.sh <name> <duration> <scenario.json> [world.sdf] [seed]
# Optional camera snapshot (needs world foiling_cam.sdf):
#   SNAPSHOT=/path/out.png SNAPSHOT_AT=20 run_phase2.sh ...
# Plant fidelity (phase 4):  PHYSICS=v2 run_phase2.sh ...   (default ref)
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh
NAME=$1; DUR=$(awk "BEGIN{printf \"%.1f\", $2}"); SCEN=$3; WORLD=${4:-foiling.sdf}; SEED=${5:-0}
bash $S/setup/stop_sim.sh
ros2 launch hydrofoil_bringup foiling.launch.py headless:=true world:=$WORLD physics:=${PHYSICS:-ref} > /tmp/foiling_launch.log 2>&1 &
sleep 6
ros2 run hydrofoil_control env_node --ros-args -p seed:=$SEED -p physics:=${PHYSICS:-ref} > /tmp/env_node.log 2>&1 &
sleep 2
if [ -n "$SNAPSHOT" ]; then
  (sleep ${SNAPSHOT_AT:-15}; timeout 20 python3 $S/scripts/save_camera_frame.py "$SNAPSHOT") &
fi
timeout $(( ${DUR%.*} + 60 )) ros2 run hydrofoil_control control_node --ros-args \
   -p scenario_json:=$SCEN -p duration:=$DUR -p log:=$S/logs/gz_phase2_$NAME.npz 2>&1 | tee /tmp/control_node.log | grep -E -A20 "FAILSAFE|saved|rror|Trace"
bash $S/setup/stop_sim.sh
grep -E "rror|Err|physics|trim" /tmp/foiling_launch.log | head -8
grep -E "set |rror|Trace" /tmp/env_node.log | cut -c1-220 | head -12
