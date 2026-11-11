# Experimental Klipper-timed capture adapter

**Status: offline validation only, not a live robot follower.** This fork contains
a separate, opt-in adapter for the experimental `robot_motion/stream` exporter
in `CrispyButtermilkChickenSalad/Klipper-ROS2-Bridge`. It does not import ROS,
load a robot model, open a network connection, publish extrusion, or execute
robot motion. No synchronized-start or hardware-safety claim is made.

Klipper is intended to own motion timing, extrusion, heaters, fans and other
auxiliary electronics; the UR16e would follow its planned toolhead motion.
The current exporter exposes **retained scheduled trapq moves and history**,
not a guaranteed complete or timely future execution queue. It is not sufficient
to drive that live architecture.
Do not run the standalone AxisForge G-code interpreter concurrently with
Klipper for the same job: that interpreter independently plans/retimes motion
and commands the Portenta extruder.

## Why the existing executor is not connected

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

## Assumed experimental protocol v1

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
the exported Klipper frame, times/durations
are seconds in `klipper_print_time`, scalar speeds are mm/s, scalar accelerations
are mm/s squared and direction components are dimensionless. At elapsed time
`t` within a phase, position is `start_position + direction * (start_velocity*t
+ acceleration*t*t/2)`. The adapter preserves direction, units, absolute times,
phase boundaries and supplied acceleration; it never rescales or retimes.
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

## Setup and offline use

Use the user's fork, not upstream:

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

Run focused tests without robot dependencies:

```bash
PYTHONPATH="$PWD/src/print_control" python3 -m unittest discover \
  -s src/print_control/test -p test_klipper_follower.py -v
```

## Blockers before any live UR follower

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
