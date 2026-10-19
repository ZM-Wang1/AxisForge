#!/bin/bash
# Keep ROS console entry points on the Python that owns the project dependencies.
set -eo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-$PROJECT_DIR/.venv/bin/python}"
if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "Project Python not found: $PYTHON_BIN. See README.md for .venv setup." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
cd -- "$PROJECT_DIR"
# Calling /usr/bin/colcon directly uses its system-Python shebang even in a venv.
exec "$PYTHON_BIN" /usr/bin/colcon build --base-paths src "$@"
