#!/usr/bin/env bash
# Build the ROS 2 workspace. Sources stay on the Windows side
# (C:\Users\ragha\hydrofoil-mpc\ros_ws\src); build/install go to the Linux
# filesystem (~/hydrofoil_build) because building on /mnt/c is slow.
set -eo pipefail
source /opt/ros/lyrical/setup.bash
SRC=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/ros_ws/src
OUT=$HOME/hydrofoil_build
mkdir -p "$OUT" && cd "$OUT"
colcon build --base-paths "$SRC" --symlink-install "$@"
echo "BUILD_DONE  (source $OUT/install/setup.bash)"
