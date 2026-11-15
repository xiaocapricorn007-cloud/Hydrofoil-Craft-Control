#!/usr/bin/env bash
# Phase 4 entry points: physics:= launch argument and the `hf physics` command.
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh; cd /tmp
bash $S/setup/stop_sim.sh
ros2 launch hydrofoil_bringup sim.launch.py headless:=true physics:=${1:-127} > /tmp/phys_cli.log 2>&1 &
sleep 20
hf() { ros2 run hydrofoil_control hf "$@"; }
echo "--- hf env (after launch)"; hf env | grep -o "physics.*"
hf physics ref; sleep 3
echo "--- hf env (after hf physics ref)"; hf env | grep -o "physics.*"
hf physics downwash,ventilation; sleep 3
echo "--- hf env (after hf physics downwash,ventilation)"; hf env | grep -o "physics.*"
hf physics warp_drive; sleep 2
echo "--- debug mask now"; timeout 5 gz topic -e -t /hydrofoil/debug -n 1 | grep -c data
bash $S/setup/stop_sim.sh
echo "--- plugin / env_node log"
grep -E "physics|trim|rejected|released|rror|Trace" /tmp/phys_cli.log | cut -c1-200
