#!/usr/bin/env bash
# Launch an Isaac Sim 5.1 script with ROS2 bridge working.
#
# Why: system ROS2 Humble is a Python 3.10 build, but Isaac Sim uses Python 3.11,
# so system rclpy fails to load. Isaac ships its own 3.11 rclpy under the bridge
# extension; we point LD_LIBRARY_PATH at it and use FastDDS (zenoh isn't installed).
#
# Usage: ./run_isaac.sh applications/spot_warehouse.py
#        ./run_isaac.sh check_ros2_nodes.py
set -e

ISAAC_REL="/home/yoon/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
BRIDGE_LIB="${ISAAC_REL}/exts/isaacsim.ros2.bridge/humble/lib"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$1" ]; then
  echo "Usage: $0 <script.py> [args...]"
  exit 1
fi

# Resolve script path: allow repo-relative or absolute.
SCRIPT="$1"; shift
if [ ! -f "$SCRIPT" ]; then
  SCRIPT="${REPO_DIR}/${SCRIPT}"
fi

# CRITICAL: .bashrc auto-sources /opt/ros/humble/setup.bash, which injects the
# system Python 3.10 site-packages into PYTHONPATH. Isaac Sim (Python 3.11) then
# picks up the wrong _rclpy_pybind11 and the bridge silently dies. Strip every
# /opt/ros path out of PYTHONPATH/AMENT/LD_LIBRARY_PATH so Isaac uses its OWN
# bundled 3.11 rclpy. We only touch THIS process's env, not the system.
strip_ros() {
  local var="$1" out=""
  local IFS=':'
  for p in ${!var}; do
    case "$p" in
      /opt/ros/*) ;;                 # drop system ROS2 paths
      *) out="${out:+$out:}$p" ;;
    esac
  done
  printf '%s' "$out"
}
export PYTHONPATH="$(strip_ros PYTHONPATH)"
export LD_LIBRARY_PATH="$(strip_ros LD_LIBRARY_PATH)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}${BRIDGE_LIB}"

echo "[run_isaac] ROS_DISTRO=$ROS_DISTRO  RMW=$RMW_IMPLEMENTATION"
echo "[run_isaac] stripped /opt/ros from PYTHONPATH/LD_LIBRARY_PATH"
echo "[run_isaac] bridge lib: $BRIDGE_LIB"
echo "[run_isaac] script: $SCRIPT"

cd "$ISAAC_REL"
exec ./python.sh "$SCRIPT" "$@"
