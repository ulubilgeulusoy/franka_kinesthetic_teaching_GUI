#!/usr/bin/env bash
set -euo pipefail
# temporarily allow unset vars for ROS setup files
set +u
source /opt/ros/humble/setup.bash
source ~/franka_ws/install/setup.bash
set -u
exec python3 "$(dirname "$0")/franka_teach_run_gui_v2.py"
