#!/bin/bash
set -eo pipefail

if [[ ! -f /workspace/src/axisforge_bringup/package.xml ]]; then
    echo 'Mount the AxisForge repository at /workspace before running axisforge-build.' >&2
    exit 1
fi
cd /workspace
# Model installation needs the mounted files and is intentionally a runtime step.
bash scripts/install_ur_models.sh
bash scripts/build.sh "$@"
echo 'Build complete. Open a new Bash session or run: source /etc/axisforge/env.bash'
