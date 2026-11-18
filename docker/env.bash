# Sourced by the entrypoint, interactive Bash, and non-interactive Bash (BASH_ENV).
# Do not enable nounset: ROS-generated setup files reference unset variables.
source /opt/ros/humble/setup.bash
if [[ -f /workspace/.docker/install/local_setup.bash ]]; then
    source /workspace/.docker/install/local_setup.bash
fi
source /opt/axisforge-venv/bin/activate
export PYTHON_BIN=/opt/axisforge-venv/bin/python
