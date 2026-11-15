#!/usr/bin/env bash
# Phase 0 acceptance checks. Run as the normal user:
#   wsl -d Ubuntu -- bash <repo>/setup/phase0_check.sh
set -o pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
pass() { echo "PASS  $1"; }
fail() { echo "FAIL  $1"; FAILED=1; }
FAILED=0

echo "--- versions"
echo "ROS_DISTRO=$ROS_DISTRO"
gz sim --versions 2>/dev/null | head -1 || fail "gz sim not found"
ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 && pass "ros_gz_bridge installed" || fail "ros_gz_bridge missing"
ros2 pkg prefix hydrofoil_bringup >/dev/null 2>&1 && pass "hydrofoil_bringup built" || fail "hydrofoil_bringup not built"

echo "--- OpenGL renderer (GUI path)"
glxinfo -B 2>/dev/null | grep -E "OpenGL renderer|OpenGL core profile version|Accelerated" || fail "glxinfo failed"

echo "--- headless physics: 5000 steps of 1 ms"
WORLD=$(ros2 pkg prefix hydrofoil_bringup)/share/hydrofoil_bringup/worlds/ocean.sdf
START=$(date +%s.%N)
if timeout 120 gz sim -s -r --iterations 5000 "$WORLD" > /tmp/gz_headless.log 2>&1; then
  END=$(date +%s.%N)
  pass "headless run OK ($(echo "$END - $START" | bc) s wall for 5.0 s sim)"
else
  fail "headless run failed (see /tmp/gz_headless.log)"; tail -5 /tmp/gz_headless.log
fi

echo "--- ROS bridge: /clock and /imu reach ROS 2"
ros2 launch hydrofoil_bringup ocean.launch.py headless:=true > /tmp/launch.log 2>&1 &
LPID=$!
sleep 12
timeout 10 ros2 topic echo /clock --once > /tmp/clock.txt 2>&1 && pass "/clock bridged" || fail "/clock not received"
timeout 10 ros2 topic echo /imu --once > /tmp/imu.txt 2>&1 && pass "/imu bridged (gravity z: $(grep -A3 linear_acceleration /tmp/imu.txt | grep ' z:' | awk '{print $2}'))" || fail "/imu not received"
RATE=$(timeout 6 ros2 topic hz /imu 2>/dev/null | grep "average rate" | tail -1)
echo "      imu $RATE"
kill -INT $LPID 2>/dev/null; sleep 2; pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null

[ $FAILED -eq 0 ] && echo "PHASE0_CHECKS_ALL_PASS" || echo "PHASE0_CHECKS_HAVE_FAILURES"
