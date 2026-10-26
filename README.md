# AxisForge

## Repository Layout

```text
src/
├── print_control/       # G-code assets, paths, print execution and keyboard control
├── extrusion_control/   # Extruder steps/mm and feed-forward extrusion profiles
├── axisforge_bringup/   # launch/ and config/, including robot calibration YAML
└── diagnostics/         # Robot and Arduino latency ROS nodes
models/ur16e/           # Custom UR16e DH and URDF classes for Robotics Toolbox
firmware/               # Arduino sketches for extrusion and latency measurements
scripts/                # Setup, build, and launch scripts for ROS 2 and UR16e
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
# Use the source YAML so parameter edits do not require another build.
ros2 launch axisforge_bringup ur_master.launch.py \
  config:="$PWD/src/axisforge_bringup/config/ur_master.yaml" \
  file:="$PWD/src/print_control/gcode/star3d.gcode" dry_run:=true
```

### Bed probing

The bed is movable: probe it again every time it is placed. With the UR driver running
(Terminal 1), jog the nozzle onto a 3x3 grid (G-code X/Y = 30, 117.5, 205) and record each
touch. Every move keeps the printing orientation and TCP from `ur_master.yaml`; each point
starts `clearance_mm` above `z_offset`, and jogs stop 3 mm below it.

```bash
cd /path/to/AxisForge   # bed_probe_file is relative to this directory
ros2 run print_control bed_probe \
  --config "$PWD/src/axisforge_bringup/config/ur_master.yaml"   # --dry-run to simulate
```

Keys: `w`/`s` Z up/down, `1`-`4` Z step 1/0.1/0.05/0.01 mm, `i`/`k` +Y/-Y and `j`/`l`
-X/+X in 10 mm steps (`--xy-step-mm`), space records (paper test; `--paper-mm` default 0.1
is subtracted), `n`/`p` next/previous point, `h` lift, `q` lift, save and quit. Ctrl-C
cancels the active trajectory. The result goes to `bed_probe_file` unless `--output` is given.

`gcode_interpreter` then fits a thin-plate spline through the probed heights (robot Base XY,
exact at every point) and prints with Z = G-code Z + bed height + `bed_z_adjust_mm`; segments
are split into <= 10 mm pieces so straight lines follow the curved bed. Points more than
`bed_probe_extrapolation_mm` outside the probed area, or a probe file taken with another TCP or
nozzle orientation, fail preflight. The startup log shows when the surface was probed.

Configuration is in [`ur_master.yaml`](src/axisforge_bringup/config/ur_master.yaml):

| Parameters | Meaning |
| --- | --- |
| `x_offset`, `y_offset`, `z_offset`, `origin_at_center`, `bed_size_mm` | Preserve the old placement convention; center mode adds half the bed dimensions to X/Y offsets |
| `bed_probe_file`, `bed_z_adjust_mm`, `bed_probe_extrapolation_mm` | Probed bed surface (relative to the launch directory; `''` uses the flat `z_offset`), first-layer fine adjust (+ raises the nozzle), and the allowed XY distance beyond the probed points |
| `nozzle_roll_deg`, `nozzle_pitch_deg`, `wrist_angle` | Fixed tool orientation in degrees; `wrist_angle` remains yaw |
| `tcp_offset_mm` | Nozzle-tip offset in tool0 coordinates; zero preserves old tool0 targets. With a measured TCP, the XYZ offsets must refer to the nozzle tip |
| `waypoint_spacing_mm` | Spatial sampling interval, replacing `split_threshold` |
| `path_tolerance_mm`, `orientation_tolerance_deg` | Model-based checks between IK samples using cubic interpolation |
| `max_joint_speed_deg_s`, `max_joint_acceleration_deg_s2` | Trajectories are stretched in time to respect these joint limits |
| `print_speed_multiplier`, `extrusion_scale_factor` | Apply before speed limiting; zero extrusion scale disables all material motion |
| `max_print_speed_mm_s`, `max_travel_speed_mm_s`, `max_extrusion_speed_mm_s` | Print, travel and filament feed limits |
| `first_layer_speed_factor`, `first_layer_height_mm` | Prefer `;LAYER:0`; use the height threshold only without layer metadata |
| `minimum_segment_duration_s` | Minimum motion duration; with look-ahead this applies to a whole run, not each source segment |
| `lookahead_enabled` | Defaults to true: combine connected spatial moves of the same extrusion sign into one action; false restores per-segment stops |
| `max_path_acceleration_mm_s2` | Path acceleration used for forward/backward look-ahead; sampled cubic TCP acceleration is also checked and retimed |
| `corner_blend_tolerance_mm` | Allowed corner rounding (default 0.05 mm); must be smaller than `path_tolerance_mm`. Zero keeps exact corners with zero junction speed |
| `max_corner_speed_mm_s` | Upper speed bound within a corner blend, further constrained by curvature and joint limits |
| `max_continuous_points` | Memory/message-size guard per run; exceeding it rejects preflight rather than silently inserting stops |
| `clearance_mm`, `approach_duration_s` | Approach above the first position and lift at completion; no automatic HOME movement |
| `extrusion_update_period_s` | Extrusion is feed-forward, as in Klipper: preflight plans filament position E(t) from the same path timing as the robot trajectory, sampled at this period and converted at a constant 185.2 steps/mm (`extrusion_calibration.STEPS_PER_MM`). From trajectory acceptance the node sends `/stepper/speed` every period so the motor reaches the planned position one period later; actual joint positions are not used. Keep the pendant speed slider at 100 % |
| `pressure_advance_s` | Klipper `pressure_advance` K for depositing motions: the planned position is E + K * dE/dt. It cancels at the rest-to-rest end, so total E is exact; the extruder briefly reverses while decelerating. 0 keeps extrusion in sync with motion |
| `extrusion_move_margin_s` | E-only moves (retract/unretract) are sent to `/stepper/move` as `[signed steps, steps/s]` and counted by the firmware; the node waits the move duration plus this margin so transport jitter cannot truncate it |
| `ignored_commands` | Explicit list of externally managed commands; every encountered code is reported |
