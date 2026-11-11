# AxisForge

## Experimental Klipper follower work

**Status: offline validation only, not a live robot follower.** This fork contains
a separate, opt-in adapter for the experimental `robot_motion/stream` exporter
in [CrispyButtermilkChickenSalad/Klipper-ROS2-Bridge](https://github.com/CrispyButtermilkChickenSalad/Klipper-ROS2-Bridge).
It does not import ROS, load a robot model, open a network connection, publish
extrusion, or execute robot motion. No synchronized-start or hardware-safety
claim is made.

Klipper is intended to own motion timing, extrusion, heaters, fans and other
auxiliary electronics; the UR16e would follow its planned toolhead motion.
The current exporter exposes **retained scheduled trapq moves and history**,
not a guaranteed complete or timely future execution queue. It is not sufficient
to drive that live architecture.
Do not run the standalone AxisForge G-code interpreter concurrently with
Klipper for the same job: that interpreter independently plans/retimes motion
and commands the Portenta extruder.

### Why the existing executor is not connected

The inspected standalone path is:

1. `gcode_interpreter.py` parses G-code and preflights the entire job.
2. `gcode_path.py` applies coordinate/bed transforms, look-ahead, IK and cubic
   joint interpolation. `retime()` and `continuous_motion()` stretch trajectory
   time to satisfy robot limits, then derive extrusion from that new timeline.
3. `motion_executor.py` sends serial, **unstamped** `FollowJointTrajectory`
   actions to `/scaled_joint_trajectory_controller/follow_joint_trajectory`.
   Execution starts at controller acceptance; host monotonic time estimates
   elapsed extrusion time. Each action waits for completion.

This cannot preserve an external Klipper timeline. The new
`print_control.klipper_follower` path does not call any of those planners or
executors. Existing standalone launch/configuration behavior is unchanged.

### Assumed experimental protocol v1

The companion `klipper_extras/robot_motion_bridge.py` registers
`robot_motion/stream` with mux `name="all"`. Its websocket batches put the
payload in `message.params`. The adapter accepts that wrapper or a bare payload
for offline capture files. It does **not** accept the subscription's initial
header/template response; capture only actual motion batch messages.

```json
{
  "params": {
    "protocol": 1,
    "sequence": 7,
    "clock": "klipper_print_time",
    "latest_motion_time": 12.25,
    "streams": {
      "toolhead": [{
        "print_time": 12.0,
        "duration": 0.25,
        "start_velocity": 20.0,
        "acceleration": 0.0,
        "start_position": [10.0, 20.0, 30.0],
        "direction": [1.0, 0.0, 0.0]
      }]
    }
  }
}
```

`streams` maps queue names to nonempty record arrays; empty notifications are
not emitted. Only `toolhead` records are interpreted. Positions are XYZ mm in
the exported Klipper frame, times/durations are seconds in `klipper_print_time`,
scalar speeds are mm/s, scalar accelerations are mm/s squared and direction
components are dimensionless. At elapsed time `t` within a phase, position is
`start_position + direction * (start_velocity*t + acceleration*t*t/2)`.
The adapter preserves direction, units, absolute times, phase boundaries and
supplied acceleration; it never rescales or retimes.
`TimedToolheadMove.sample(print_time)` evaluates Cartesian position, velocity
and acceleration only within that phase.

Extruder queues use their native extrusion coordinates, **not Cartesian XYZ**.
They are ignored, never fused with toolhead records or sent to the Portenta.
They still count toward the per-batch record bound. `latest_motion_time` is
required and must be finite/nonnegative but is informational: it is the maximum
record end in that batch, not a host-clock timestamp, execution deadline,
synchronization anchor, or completeness guarantee.
Additional fields are ignored; unsupported protocol versions/clocks are rejected.

These are assumptions about an experimental exporter, not a finalized public
API. Confirm them against the exact custom Klipper build before recording.
There is no network/subscription implementation here and no HTTP G-code path.

### Setup and offline use

Use this fork, not upstream:

```bash
git clone https://github.com/CrispyButtermilkChickenSalad/AxisForge.git
cd AxisForge
# Save actual motion batch messages as one JSON object per line; no pretty-printing.
# This module needs only Python's standard library, not ROS or the UR driver.
PYTHONPATH="$PWD/src/print_control" python3 -m print_control.klipper_follower \
  --experimental-klipper-follower --input /path/to/capture.ndjson
```

The JSON example above can be saved on a single line to smoke-test the format.
It is a synthetic format example, not a robot trajectory. After the normal ROS
workspace build (`./scripts/build.sh` and `source install/setup.bash`), the same
offline command is available as:

```bash
ros2 run print_control klipper_follower \
  --experimental-klipper-follower --input /path/to/capture.ndjson
```

The command is disabled without the explicit flag. It prints a JSON validation
summary only after the entire file passes, with `mode=capture_validation_only`
and `live_execution_available=false`. Invalid input produces stderr and exit 1.
There is deliberately no `--execute` switch or live launch integration.

### Buffer limits, sequence handling and recovery

Default limits are 1 MiB per NDJSON line (`--max-line-bytes`), 4096 records per
batch/pending toolhead FIFO (`--max-records`) and a 10-second pending time span
including gaps (`--max-buffer-s`). Oversized input is rejected, not truncated.
Offline validation drains each batch immediately, so the horizon is per pending
batch there; API consumers that do not drain are bounded across batches too.

`KlipperCaptureBuffer(experimental=True)` exposes `ingest(message)`, `pop()` and
`len(buffer)` for hardware-free consumers. The first sequence may be any
nonnegative integer, allowing partial captures; every following batch must
increment by exactly one, including batches containing only extruder records.
The exporter's first sequence after the shared helper starts is 1, but a client
joining an existing subscription can observe a higher first sequence. There is
no epoch identifier when the helper restarts after all clients disconnect.
Queue timestamps must not overlap or regress, even after draining (1 ns
comparison tolerance for floating-point arithmetic; timestamps are never snapped).
Gaps are preserved and counted in the CLI summary, not interpolated into a
hold or motion. No Cartesian continuity, reachability or collision check is
performed. Missing records cannot be reconstructed.

Zero-duration position-set markers are rejected: without an explicit marker
type or guaranteed delivery, the adapter cannot interpret them as safe coordinate
resets. The exporter's cursor does not guarantee deduplication, gap-free coverage
or continuity after queue edits/reconnects.

Ingestion is transactional. A sequence gap/reset/duplicate, malformed phase,
overflow, overlap or capacity violation latches a fault and makes pending data
unavailable. `pop()` on an empty buffer also faults. There is no automatic
retry, eviction or sequence resynchronization: discard the adapter and start a
new capture/session explicitly. The CLI treats end-of-file as offline completion,
not proof that a live job has ended. A file with no toolhead phases is rejected.
No wall-clock freshness or live underrun deadline is inferred from this capture.

### Testing the adapter

Run focused tests without robot dependencies:

```bash
PYTHONPATH="$PWD/src/print_control" python3 -m unittest discover \
  -s src/print_control/test -p test_klipper_follower.py -v
```

The implementation was checked with 15 focused unit tests, Python compilation
and a subprocess smoke test of the documented CLI, including default-disabled
behavior and rejection of `--execute`. These are offline checks only; ROS/UR
runtime, custom Klipper behavior, calibration and hardware synchronization have
not been validated.

### Blockers before any live UR follower

The following need an agreed interface and live validation, not an enable flag:

1. A **future** Klipper motion schedule with guaranteed look-ahead/lead time,
   backpressure, job identity, start barrier, reset/reconnect semantics, idle/end
   markers and a coordinated stop mechanism. A diagnostic mixture of retained
   scheduled moves/history and sequence numbers alone cannot provide these.
   Test the exporter on the actual custom Klipper build; records may already be
   too late when received.
2. A measured Klipper print-time to ROS/controller-clock mapping, with bounded
   offset/drift, timestamp age and transport jitter. Define late-data and buffer
   underrun deadlines. Neither a realtime kernel nor starting two jobs together
   establishes synchronization.
3. A UR controller interface for continuous, timestamped trajectory consumption
   without stop/start action gaps or silent speed scaling. Validate acceptance
   latency, trajectory replacement, feedback and protective-stop behavior.
   The existing scaled action executor is not such an interface.
4. Calibrated Klipper-frame to robot-base/TCP/orientation mapping (including bed
   mesh/leveling/kinematic transforms already applied by Klipper). Do not apply
   AxisForge bed compensation twice. The exporter carries no orientation/A-axis
   or robot redundancy contract. AxisForge's nominal Robotics Toolbox UR16e model
   is not the driver-specific calibration loaded by `scripts/ur_ros2_driver.sh`;
   neither model agreement nor calibration has been validated by this change.
5. Timing-preserving IK/Jacobian conversion and model/controller interpolation
   checks against joint speed, acceleration, singularity, workspace and collision
   constraints. Reject infeasible Klipper timing upstream or replan **at Klipper**;
   AxisForge must not independently stretch a follower trajectory.
6. Cross-device stop/pause/fault recovery for robot, extrusion and auxiliaries,
   plus measured tracking error on the actual UR16e under load. Start with
   simulation/captured data and no extrusion; hardware trials require the user's
   established robot safety process.

This adapter is a non-hardware-dependent starting point, not calibration,
hardware validation, a simultaneous-job runner, or permission to operate a robot.

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
git clone https://github.com/CrispyButtermilkChickenSalad/AxisForge.git
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
touch. Probing always uses an upright nozzle (roll 180°, pitch 0°), with the printing yaw
and TCP from `ur_master.yaml`; each point starts `clearance_mm` above `z_offset`, and jogs
stop 3 mm below it.

```bash
cd /path/to/AxisForge   # bed_probe_file is relative to this directory
ros2 run print_control bed_probe \
  --config "$PWD/src/axisforge_bringup/config/ur_master.yaml"   # --dry-run to simulate
```

Keys: `w`/`s` Z up/down, `1`-`4` Z step 1/0.1/0.05/0.01 mm, `i`/`k` +Y/-Y and `j`/`l`
-X/+X in 10 mm steps (`--xy-step-mm`), space records (paper test; `--paper-mm` default 0.1
is subtracted), `n`/`p` next/previous point, `h` lift, `q` lift, save and quit. Ctrl-C
cancels the active trajectory. The result goes to `bed_probe_file` unless `--output` is given.


Configuration is in [`ur_master.yaml`](src/axisforge_bringup/config/ur_master.yaml):

| Parameters | Meaning |
| --- | --- |
| `x_offset`, `y_offset`, `z_offset`, `origin_at_center`, `bed_size_mm` | Preserve the old placement convention; center mode adds half the bed dimensions to X/Y offsets |
| `bed_probe_file`, `bed_z_adjust_mm`, `bed_probe_extrapolation_mm` | Probed bed surface (relative to the launch directory; `''` uses the flat `z_offset`), first-layer fine adjust (+ raises the nozzle), and the allowed XY distance beyond the probed points |
| `nozzle_roll_deg`, `nozzle_pitch_deg`, `wrist_angle` | Initial tool orientation in degrees; G-code A overrides pitch, and `wrist_angle` remains yaw. Bed probing always uses roll 180°, pitch 0° |
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
| `extrusion_update_period_s` | Preflight plans extrusion E(t) from robot trajectory timing and converts it to stepper pulses. During execution, `/stepper/speed` receives the next-period average feed rate, with gradual correction for host scheduling delays. Actual motor position and transport latency are not measured. Keep the UR speed slider at 100% |
| `pressure_advance_s` | Klipper `pressure_advance` K for depositing motions: the planned position is E + K * dE/dt. It cancels at the rest-to-rest end, so total E is exact; the extruder briefly reverses while decelerating. 0 keeps extrusion in sync with motion |
| `extrusion_move_margin_s` | E-only moves (retract/unretract) are sent to `/stepper/move` as `[signed steps, steps/s]` and counted by the firmware; the node waits the move duration plus this margin so transport jitter cannot truncate it |
| `ignored_commands` | Explicit list of externally managed commands; every encountered code is reported |
