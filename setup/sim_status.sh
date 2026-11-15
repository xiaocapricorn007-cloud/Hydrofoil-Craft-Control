#!/usr/bin/env bash
# Quick health check of a running simulator.
S=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source $S/setup/env.sh; cd /tmp
grep -E "ready|trim|released|rror|Trace" /tmp/sim_gui.log | cut -c1-170 | head
echo "--- camera:"; timeout 5 gz topic -e -t /gui/currently_tracked -n 1 | head -2
echo "--- craft:"; timeout 15 ros2 run hydrofoil_control hf status
