#!/bin/bash

# Serial bridge for the UR16e printing system's Portenta extruder.
# The Agent is robot-independent; the firmware subscribes to /stepper/speed.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MICROROS_WS="${MICROROS_WS:-$SCRIPT_DIR/../.micro_ros_ws}"

# Use Humble's system Python instead of an active Conda interpreter.
export PATH="/opt/ros/humble/bin:/usr/bin:/bin:$PATH"
source /opt/ros/humble/setup.bash
if [[ -f "$MICROROS_WS/install/local_setup.bash" ]]; then
    source "$MICROROS_WS/install/local_setup.bash"
fi

if ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
    echo "micro_ros_agent is not installed. See README section 6 or set MICROROS_WS." >&2
    exit 1
fi

DEVICE="${1:-/dev/ttyACM0}"
if [[ $# -gt 0 ]]; then
    shift
fi
if [[ ! -c "$DEVICE" ]]; then
    echo "Serial device not found: $DEVICE. Connect the board or pass its device path." >&2
    exit 1
fi
if [[ ! -r "$DEVICE" || ! -w "$DEVICE" ]]; then
    echo "Serial access denied: $DEVICE. Add your user to dialout, then log out and back in." >&2
    exit 1
fi

exec ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEVICE" "$@"
