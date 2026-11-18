# syntax=docker/dockerfile:1
# Pin the previously verified Humble base so the tag can move independently.
FROM ros:humble-ros-base-jammy@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG DEV_UID=1000
ARG DEV_GID=1000

# Humble's binary packages use Ubuntu 22.04's system Python 3.10.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential ca-certificates curl gdb git less nano \
        python3-colcon-common-extensions python3-dev python3-pip \
        python3-pytest python3-rosdep python3-venv \
        ros-humble-ur-robot-driver ros-humble-ur-description \
    && rm -rf /var/lib/apt/lists/*

# .dockerignore exposes only package.xml files under src/. This temporary,
# read-only build mount installs ROS dependencies without adding source to layers.
# ament_python is a build type, not a Humble rosdep key; colcon's Python/ROS
# extensions and setuptools provide it (installed above and in the venv below).
RUN --mount=type=bind,source=src,target=/tmp/ros-packages \
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
    && rosdep update --rosdistro=humble \
    && apt-get update \
    && rosdep install --from-paths /tmp/ros-packages --ignore-src --rosdistro=humble \
        --skip-keys=ament_python -y \
    && rm -rf /var/lib/apt/lists/*

RUN if ! getent group "$DEV_GID" >/dev/null; then groupadd --gid "$DEV_GID" developer; fi \
    && useradd --uid "$DEV_UID" --gid "$DEV_GID" --create-home --shell /bin/bash developer \
    && /usr/bin/python3 -m venv --system-site-packages /opt/axisforge-venv \
    && chown -R "$DEV_UID:$DEV_GID" /opt/axisforge-venv

# Only dependency manifests and container support files are copied into the image.
COPY requirements.txt docker/constraints.txt docker/requirements-dev.txt /tmp/axisforge-deps/
USER developer
# Keep setup.py develop compatible with Humble/colcon. Build native extensions
# against the project's NumPy 1.x, including when no wheel is available.
RUN /opt/axisforge-venv/bin/python -m pip install --no-cache-dir \
        pip==24.0 setuptools==65.5.1 wheel==0.43.0 numpy==1.26.4 \
    && /opt/axisforge-venv/bin/python -m pip install --no-cache-dir --no-build-isolation \
        -r /tmp/axisforge-deps/requirements.txt \
        -r /tmp/axisforge-deps/requirements-dev.txt \
        -c /tmp/axisforge-deps/constraints.txt

USER root
COPY docker/env.bash docker/colcon-defaults.yaml /etc/axisforge/
COPY --chmod=755 docker/entrypoint.sh /usr/local/bin/axisforge-entrypoint
COPY --chmod=755 docker/build.sh /usr/local/bin/axisforge-build
RUN printf '\nsource /etc/axisforge/env.bash\n' >> /etc/bash.bashrc

ENV VIRTUAL_ENV=/opt/axisforge-venv \
    PYTHON_BIN=/opt/axisforge-venv/bin/python \
    PATH=/opt/axisforge-venv/bin:$PATH \
    BASH_ENV=/etc/axisforge/env.bash \
    COLCON_DEFAULTS_FILE=/etc/axisforge/colcon-defaults.yaml \
    PYTHONUNBUFFERED=1 \
    MPLBACKEND=Agg

USER developer
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/axisforge-entrypoint"]
CMD ["sleep", "infinity"]
