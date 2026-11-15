#!/usr/bin/env bash
# Phase 1 parity run:  run_parity.sh <controller> <out.npz> [duration]
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh
CTRL=${1:-MPC-IDP}; OUT=${2:-$S/logs/gz_step_$CTRL.npz}; DUR=${3:-4.0}
bash $S/setup/stop_sim.sh
ros2 launch hydrofoil_bringup foiling.launch.py headless:=true > /tmp/foiling_launch.log 2>&1 &
LPID=$!
sleep 6
timeout 120 ros2 run hydrofoil_control parity_node --ros-args -p controller:=$CTRL -p scenario:=step \
   -p duration:=$DUR -p out:=$OUT 2>&1 | grep -v "^$"
bash $S/setup/stop_sim.sh
grep -E "FoilDynamics|rror|Err" /tmp/foiling_launch.log | head -8
