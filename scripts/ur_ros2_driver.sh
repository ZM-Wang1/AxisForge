#!/bin/bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CALIBRATION_FILE="${UR_CALIBRATION_FILE:-$SCRIPT_DIR/../src/axisforge_bringup/config/ur16e_calibration.yaml}"
if [[ ! -r "$CALIBRATION_FILE" ]]; then
    echo "UR16e calibration file not found: $CALIBRATION_FILE" >&2
    exit 1
fi
source /opt/ros/humble/setup.bash

exec ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur16e \
    robot_ip:="${ROBOT_IP:-192.168.0.100}" \
    kinematics_params_file:="$CALIBRATION_FILE" \
    launch_rviz:=false "$@"
