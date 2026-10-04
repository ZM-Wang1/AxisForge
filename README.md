# AxisForge

## Repository Layout

```text
src/
├── ur_control/          # UR robot control; custom Toolbox models in models/
├── print_control/       # G-code assets, paths, print execution and keyboard control
├── extrusion_control/   # Extruder calibration and progress-based extrusion
├── axisforge_bringup/   # launch/ and config/, including robot calibration YAML
└── diagnostics/         # Robot and Arduino latency ROS nodes
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
sudo apt install python3-dev python3-pip

# Install required packages from requirements.txt
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ZM-Wang1/AxisForge.git
cd AxisForge
pip install -r requirements.txt
```

### 5. Custom UR16e Models Installation

Install the custom UR16e DH and URDF models into the project's `.venv`:
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
.venv/bin/python /usr/bin/colcon build --base-paths src
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
```

### Straight-Line Printing Test with a Fixed Nozzle Angle

Each run prints one straight line with a fixed end-effector orientation. Parameters are configured in
[`src/axisforge_bringup/config/line_print_test.yaml`](src/axisforge_bringup/config/line_print_test.yaml).
Edit the YAML file and rerun the test to apply changes. This test uses a separate entry point; do not run
`ur_master.launch.py`, the G-code executor, or other trajectory-sending nodes at the same time.

Rebuild before using the new entry point for the first time:

```bash
cd /path/to/AxisForge
source /opt/ros/humble/setup.bash
source .venv/bin/activate
.venv/bin/python /usr/bin/colcon build --base-paths src
source install/setup.bash

# Explicit preview: the source YAML may contain dry_run: false from hardware tests.
# Parameter-only edits do not require rebuilding when this source YAML is used.
ros2 run print_control line_print_test --ros-args \
  --params-file src/axisforge_bringup/config/line_print_test.yaml -p dry_run:=true
```

The launch entry point is `ros2 launch axisforge_bringup line_print_test.launch.py`.

Common parameters are listed below. Keep `.0` for floating-point parameters, and provide exactly three numbers for 3D parameters:

| Parameter | Units / Meaning | Example |
| --- | --- | --- |
| `line_length_mm` | Line length, mm | `30.0` |
| `nozzle_rpy_deg` | Absolute roll, pitch, and yaw of tool0, degrees | `[180.0, 0.0, 90.0]` |
| `filament_per_mm` | Filament mm per mm of printed path | `0.20` is an example; `0.0` disables extrusion |
| `print_speed_mm_s` | Nominal average printing speed, mm/s | `5.0` |
| `start_xyz_mm` | Nozzle tip starting position, XYZ in the robot Base frame, mm | Enter the measured position |
| `line_direction_deg` | Printing direction in the Base XY plane | `0.0` along +X, `90.0` along +Y |
| `tcp_offset_mm` | Offset from tool0 to the nozzle tip in the tool0 frame, mm | Enter the measured mounting offset |
| `clearance_mm` | Lift distance above the starting point and after printing, along Base +Z | `10.0` |
| `travel_speed_mm_s` | Lowering and lifting speed, mm/s | `5.0` |
| `approach_duration_s` | Duration of the joint motion from the current configuration to above the starting point, s | `8.0` |
| `waypoint_spacing_mm` | Cartesian path sampling interval, mm | `1.0` |
| `dry_run` | `true` plans only; `false` executes one test | Default: `true` |
