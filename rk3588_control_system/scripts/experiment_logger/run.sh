#!/bin/bash
set -e
LOGGER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${MANTA_ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS_WORKSPACE="${MANTA_ROS_WORKSPACE:-${HOME}/usv_ws}"
if [ -f "$ROS_SETUP" ]; then
  source "$ROS_SETUP"
fi
if [ -f "$ROS_WORKSPACE/install/setup.bash" ]; then
  source "$ROS_WORKSPACE/install/setup.bash"
fi
cd "$LOGGER_DIR"
exec "${MANTA_LOGGER_PYTHON:-python3}" -u experiment_logger.py --config config.json "$@"
