#!/usr/bin/env bash
# Phase 0: ROS 2 Lyrical + Gazebo Jetty (gz-sim 10) on Ubuntu 26.04 (WSL2).
# Run as root:  wsl -d Ubuntu -u root -- bash <repo>/setup/phase0_install.sh
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
VER=1.3.0
CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
echo "== [1/5] ROS 2 apt source ($VER, $CODENAME)"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${CODENAME}_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb
fi
echo "== [2/5] apt update"
apt-get update -q
echo "== [3/5] install ROS 2 Lyrical desktop, ros_gz (Gazebo Jetty), build tools"
apt-get install -y -q --no-install-recommends \
  ros-lyrical-desktop ros-lyrical-ros-gz ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep python3-numpy python3-scipy \
  mesa-utils python3-pip
echo "== [4/5] osqp (QP solver used by the MPC), system-wide"
python3 -c "import osqp" 2>/dev/null || pip3 install --break-system-packages osqp
echo "== [5/5] rosdep init"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init || true
echo "PHASE0_INSTALL_DONE"
