#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP_FILE="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP_FILE="/home/parc/franka_ws/install/setup.bash"

if [[ ! -f "$ROS_SETUP_FILE" ]]; then
  echo "Missing ROS setup file: $ROS_SETUP_FILE" >&2
  exit 1
fi

if [[ ! -f "$WORKSPACE_SETUP_FILE" ]]; then
  echo "Missing workspace setup file: $WORKSPACE_SETUP_FILE" >&2
  exit 1
fi

# Temporarily allow unset vars for ROS setup files.
set +u
source "$ROS_SETUP_FILE"
source "$WORKSPACE_SETUP_FILE"
set -u

cd "$SCRIPT_DIR"
exec python3 "$SCRIPT_DIR/franka_teach_run_gui_v2.py"
