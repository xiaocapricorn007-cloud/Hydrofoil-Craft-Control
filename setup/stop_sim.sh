#!/usr/bin/env bash
# Stop every simulator process (Gazebo server/GUI, bridge, control nodes).
for pat in "gz sim" "gz-sim" "parameter_bridge" "control_node" "parity_node" "ros2 launch hydrofoil_bringup"; do
  pkill -INT -f "$pat" 2>/dev/null
done
sleep 2
for pat in "gz sim" "gz-sim" "parameter_bridge" "control_node" "parity_node" "ros2 launch hydrofoil_bringup"; do
  pkill -KILL -f "$pat" 2>/dev/null
done
sleep 0.5
