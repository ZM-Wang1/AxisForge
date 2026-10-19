# AxisForge

## Repository Layout

```text
src/
├── print_control/       # G-code assets, paths, print execution and keyboard control
├── extrusion_control/   # Extruder calibration and progress-based extrusion
├── axisforge_bringup/   # launch/ and config/, including robot calibration YAML
└── diagnostics/         # Robot and Arduino latency ROS nodes
models/ur16e/           # Custom UR16e DH and URDF classes for Robotics Toolbox
firmware/               # Arduino sketches for extrusion and latency measurements
STL/                    # Printable example meshes
```

## Installation and Setup

This guide provides step-by-step instructions for setting up the complete software environment.

### Prerequisites

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- Ubuntu Pro account (free for personal use) for RT kernel access

### 1. Real-Time Kernel Installation

For optimal robot control performance, install the RT kernel via Ubuntu Pro:
```bash
# Sign up for Ubuntu Pro and attach PC
sudo pro attach <your-token>

# Enable RT kernel
sudo pro enable realtime-kernel

# Reboot system
sudo reboot
```

### 2. ROS 2 Humble Installation
```bash
# Set up sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 Humble
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. UR ROS 2 Driver Installation
```bash
sudo apt-get install ros-humble-ur
```

### 4. Python Dependencies Installation
```bash
# Install Python development tools
sudo apt install python3-dev python3-pip python3-venv

# Install required packages from requirements.txt
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ZM-Wang1/AxisForge.git
cd AxisForge
# Keep ROS packages visible while installing project dependencies in .venv.
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

### 5. Custom UR16e Models Installation

Install the custom UR16e DH and URDF models from [`models/`](models/README.md)
into the project's `.venv`:
```bash
./scripts/install_ur_models.sh
```

### 6. Micro-ROS Setup

For Arduino Portenta H7:

1. Install Arduino IDE 2.x from https://www.arduino.cc/en/software
2. Add Arduino Mbed OS Portenta Boards via Board Manager
3. The micro-ROS sketch uses the Portenta core's hardware timer support; install `AccelStepper` only for the standalone serial sketch
4. Download latest `micro_ros_arduino` for Humble from [Releases](https://github.com/micro-ROS/micro_ros_arduino/releases)
5. Upload the downloaded ZIP to the IDE using `Sketch -> Include library -> Add .ZIP Library...`
6. Select the Portenta H7 Main Core (M7) target and upload `firmware/DRV8825_microros/DRV8825_microros.ino`
7. Install micro-ROS agent on PC: https://github.com/micro-ROS/micro_ros_setup

### 7. Build the Workspace

```bash
cd /path/to/AxisForge
source /opt/ros/humble/setup.bash
source .venv/bin/activate
# Build entry points with the Python environment containing the UR16e models.
./scripts/build.sh
source install/setup.bash
```

`scripts/build.sh` explicitly runs colcon with `.venv/bin/python` and accepts build
options such as `--packages-select print_control`. To use another Python environment,
set `PYTHON_BIN` to its absolute interpreter path, as with `install_ur_models.sh`.

Avoid plain `colcon build` when colcon comes from `/usr/bin/colcon`: its system-Python
shebang takes precedence even with `(.venv)` active, so the generated ROS entry points
use `/usr/bin/python3` and cannot find dependencies installed only in `.venv`.
If launch fails with `ModuleNotFoundError: No module named 'roboticstoolbox'`, rerun
`./scripts/build.sh` and source `install/setup.bash`. The equivalent explicit command
is `.venv/bin/python /usr/bin/colcon build --base-paths src`; rebuilding replaces the
incorrect entry points without deleting the workspace or installing into system Python.

### Usage

```bash
# Terminal 1: Start the UR16e driver
cd /path/to/AxisForge
./scripts/ur_ros2_driver.sh

# Terminal 2: Start the serial bridge to the Portenta extruder
cd /path/to/AxisForge
./scripts/microros.sh

# Terminal 3: Launch control system
cd /path/to/AxisForge
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch axisforge_bringup ur_master.launch.py
```

### G-code Toolpath Planning and Execution

`ur_master.launch.py` now runs one G-code executor. It parses the entire file into
segments, transforms coordinates, samples the nozzle path, solves IK and checks
the interpolated joint trajectory before sending any motion. It uses
`/scaled_joint_trajectory_controller/follow_joint_trajectory` directly.

Rebuild `extrusion_control`, `print_control` and `axisforge_bringup` after updating.
The default command still executes `star3d.gcode`. For offline preflight:

```bash
ros2 launch axisforge_bringup ur_master.launch.py dry_run:=true

# Use the source YAML so parameter edits do not require another build.
ros2 launch axisforge_bringup ur_master.launch.py \
  config:="$PWD/src/axisforge_bringup/config/ur_master.yaml" \
  file:="$PWD/src/print_control/gcode/star3d.gcode" dry_run:=true
```

Remove `dry_run:=true` to execute using fresh robot joint feedback. Preflight uses
`dry_run_seed_deg` only in dry-run mode, and creates no hardware command publishers
in that mode. Large files can take several minutes to preflight. All segments must
pass before the first motion; an unreachable point reports its original G-code line.

Configuration is in [`ur_master.yaml`](src/axisforge_bringup/config/ur_master.yaml):

| Parameters | Meaning |
| --- | --- |
| `x_offset`, `y_offset`, `z_offset`, `origin_at_center`, `bed_size_mm` | Preserve the old placement convention; center mode adds half the bed dimensions to X/Y offsets |
| `nozzle_roll_deg`, `nozzle_pitch_deg`, `wrist_angle` | Fixed tool orientation in degrees; `wrist_angle` remains yaw |
| `tcp_offset_mm` | Nozzle-tip offset in tool0 coordinates; zero preserves old tool0 targets. With a measured TCP, the XYZ offsets must refer to the nozzle tip |
| `waypoint_spacing_mm` | Spatial sampling interval, replacing `split_threshold` |
| `path_tolerance_mm`, `orientation_tolerance_deg` | Model-based checks between IK samples using cubic interpolation |
| `max_joint_speed_deg_s`, `max_joint_acceleration_deg_s2` | Trajectories are stretched in time to respect these joint limits |
| `print_speed_multiplier`, `extrusion_scale_factor` | Apply before speed limiting/calibration; zero extrusion scale disables all material motion |
| `max_print_speed_mm_s`, `max_travel_speed_mm_s`, `max_extrusion_speed_mm_s` | Print, travel and filament feed limits |
| `first_layer_speed_factor`, `first_layer_height_mm` | Prefer `;LAYER:0`; use the height threshold only without layer metadata |
| `minimum_segment_duration_s` | Minimum motion duration; with look-ahead this applies to a whole run, not each source segment |
| `lookahead_enabled` | Defaults to true: combine connected spatial moves of the same extrusion sign into one action; false restores per-segment stops |
| `max_path_acceleration_mm_s2` | Path acceleration used for forward/backward look-ahead; sampled cubic TCP acceleration is also checked and retimed |
| `corner_blend_tolerance_mm` | Allowed corner rounding (default 0.05 mm); must be smaller than `path_tolerance_mm`. Zero keeps exact corners with zero junction speed |
| `max_corner_speed_mm_s` | Upper speed bound within a corner blend, further constrained by curvature and joint limits |
| `max_continuous_points` | Memory/message-size guard per run; exceeding it rejects preflight rather than silently inserting stops |
| `trajectory_tracking_window_mm` | Local arc-length search around controller trajectory time for actual TCP projection; keep larger than expected tracking/interpolation error |
| `clearance_mm`, `approach_duration_s` | Approach above the first position and lift at completion; no automatic HOME movement |
| `extrusion_smoothing_s`, `extrusion_update_period_s`, `extrusion_feedback_timeout_s` | Actual-progress velocity averaging, command interval and stale-feedback timeout |
| `ignored_commands` | Explicit list of externally managed commands; every encountered code is reported |

Supported commands include G0/G1 (including compact and modal coordinates), G20/G21,
G90/G91, independent E modes M82/M83, G92, G4, M220/M221 and M400. The modal behavior
follows Marlin: G90/G91 reset both modes, and a subsequent M82/M83 overrides E only.
Unsupported commands such as G2/G3 fail preflight instead of silently changing state.
The default compatibility list ignores G28, heater/fan commands and M84, as the old
entry point did; it does not home, heat, cool or wait for temperature. Those devices
remain externally managed. Remove a code from the list to reject it.

Look-ahead is enabled by default. Adjacent spatial moves with the same extrusion
sign share one trajectory action, including continuous joint velocities through
internal boundaries. A backward braking pass and forward acceleration pass slow
down before corners or lower-speed sections. Quadratic corner transitions stay
within the configured geometric tolerance, and the commanded joint interpolation
is checked against the original G-code path. Joint and sampled TCP limits can further
slow the whole run. Near reversals and zero-blend corners still reach zero speed,
but do not introduce an action handoff. E-only commands, dwell/M400, direction changes
and print/travel transitions remain explicit stop boundaries.

The executor tracks cumulative material over each entire run, conserving source E
when a corner is rounded. Crossing a source segment does not reset the velocity
history or publish zero extrusion. Actual joint feedback determines nozzle progress;
the controller's `desired.time_from_start` only identifies the local path branch,
so returning to the same point on a closed path does not jump to the wrong segment.
The controller must report this trajectory time correctly, including speed scaling.
Signed step rate follows measured material progress and the existing calibration.

E-only actions use interruptible host timing and always publish zero on completion.
This is not encoder-based material control or exact pulse counting: feedback,
smoothing and transport delays affect delivered material. Residual nozzle pressure
can still cause accumulation at necessary stops; this change does not implement
pressure advance. Reverse motion uses the same calibration table at the absolute
feed rate. The model's link-origin height check is not full collision detection.

Ctrl+C, `/kb/shutdown` and `/kb/pause` stop extrusion and cancel the active trajectory
in place. Pause currently ends the run; it does not create a resumable checkpoint.
Old live speed/flow/Z edits and next-step requests are rejected with a message, since
they would invalidate the completed preflight. Change parameters between runs.

Straight-line tests also use G-code through `ur_master.launch.py` and the same
`ur_master.yaml` configuration. Supply the test file with `file:=/path/to/test.gcode`;
G0/G1 coordinates define its geometry, F defines feed rate, and E defines filament
movement. Use `dry_run:=true` to preflight it before execution.
