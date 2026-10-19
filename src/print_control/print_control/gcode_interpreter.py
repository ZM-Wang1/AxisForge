"""Preflight G-code toolpaths and execute sampled UR trajectories with extrusion."""

from dataclasses import fields, replace
from pathlib import Path
import math
import signal
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32

from extrusion_control.extrusion_calibration import steps_per_mm
from extrusion_control.progress_extrusion import ProgressExtrusion
from print_control.gcode_parser import GCodeParser
from print_control.gcode_path import GCodeConfig, GCodePlanner
from print_control.motion_executor import MotionExecutor
from print_control.motion_planner import MotionConfig


class GCodeInterpreter(MotionExecutor):
    """Separate parsing, complete path planning, and acknowledged execution."""

    def __init__(self):
        """Read configuration; dry runs have no motion or extrusion publishers."""
        super().__init__('gcode_interpreter')
        self.settings = GCodeConfig(**self.read_defaults(GCodeConfig()))
        self.settings.validate()
        # Geometry and material come from G-code; orientation comes from GCodeConfig.
        supplied = {'path_length_mm', 'nozzle_rpy_deg', 'filament_per_mm'}
        config = MotionConfig(**self.read_defaults(MotionConfig(), excluded=supplied))
        planner = GCodePlanner(self.settings, config)
        self.active_path = None
        self.active_segment = None
        self.initialize_execution(planner.config, planner)
        self.base_config = self.config
        self.last_planning_log = 0.0
        legacy = self.declare_parameter('split_threshold', -1.0).value
        if legacy != -1.0:
            raise ValueError('split_threshold was replaced by waypoint_spacing_mm; update YAML')
        self.create_subscription(Bool, '/kb/shutdown', self.request_stop, 10)
        self.create_subscription(Bool, '/kb/pause', self.request_stop, 10)
        for topic in ('speed_multiplier', 'extrusion_scale', 'adjust_z_offset'):
            self.create_subscription(Float32, f'/kb/{topic}', self.reject_live_edit, 10)
        self.create_subscription(Bool, '/kb/next_step', self.reject_live_edit, 10)

    def read_defaults(self, defaults, excluded=()):
        """Declare only parameters relevant to this entry point."""
        values = {}
        for field in fields(defaults):
            if field.name in excluded:
                continue
            default = getattr(defaults, field.name)
            if isinstance(default, tuple):
                default = list(default)
            values[field.name] = self.declare_parameter(
                field.name, default, ParameterDescriptor(read_only=True),
            ).value
        return values

    def steps_per_mm(self, extrusion_speed_mmps):
        """Expose the shared calibration for callers of the previous interpreter."""
        return steps_per_mm(extrusion_speed_mmps)

    def request_stop(self, message):
        """Stop in place and cancel; never skip a segment or return HOME implicitly."""
        if message.data:
            self.stop_requested = True
            self.set_extrusion(0.0, force=True)

    def reject_live_edit(self, message):
        """Preflighted geometry and feed settings change only between runs."""
        self.get_logger().warn(
            'Live speed/flow/Z edits and next-step are disabled for a preflighted path. '
            'Stop, change the configuration, and launch again.'
        )

    def planning_progress(self, index, total):
        """Keep cancellation responsive and report bounded planning progress."""
        self.check_interrupted()
        now = time.monotonic()
        if now - self.last_planning_log > 2.0:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.check_interrupted()
            self.get_logger().info(f'Preflighting segment {index + 1}/{total}')
            self.last_planning_log = now

    def configure_step(self, step):
        """Bind the motion executor to this step's material and geometry."""
        segment = step.segment
        self.extrusion_direction = 1.0
        self.config = self.base_config
        self.active_path = step.path
        self.active_segment = segment
        if segment is None or step.motion is None:
            return
        if step.path is not None:
            length = step.path.length_mm
            self.config = replace(
                self.base_config, path_length_mm=length,
                filament_per_mm=float(step.path.material[-1]) / length,
                progress_threshold_mm=min(self.base_config.progress_threshold_mm, length / 10),
            )
            self.extrusion_direction = step.path.direction
            return
        length = segment.length_mm
        self.config = replace(
            self.base_config, path_length_mm=length,
            filament_per_mm=abs(segment.delta_e) / length,
            progress_threshold_mm=min(self.base_config.progress_threshold_mm, length / 10),
        )
        self.extrusion_direction = math.copysign(1.0, segment.delta_e)

    def reset_extrusion(self):
        """Reset once per action; retain material history across all source boundaries."""
        super().reset_extrusion()
        self.last_desired_time = None
        self.controller_clock_started = False
        if self.active_path is not None and self.active_path.material[-1] > 0:
            self.extrusion_sync = ProgressExtrusion(
                1.0, float(self.active_path.material[-1]), self.config.extrusion_smoothing_s,
            )

    def on_feedback(self, message, printing):
        """Allow UR's first feedback cycle to replace the preceding desired state."""
        if self.active_path is not None and printing and not self.controller_clock_started:
            # The scaled UR controller reports last_commanded_state_ as desired on
            # its first cycle. Its time may still belong to the previous action.
            self.controller_clock_started = True
            self.feedback_received = time.monotonic()
            return
        super().on_feedback(message, printing)

    def feedback_progress(self, joints, feedback):
        """Use actual TCP progress; controller time only selects the path branch."""
        path = self.active_path
        if path is None:
            segment = self.active_segment
            start, end = np.asarray(segment.start), np.asarray(segment.end)
            self.planner.check_pose(joints, start, end)
            position = self.planner.nozzle_position(joints)
            direction = (end - start) / segment.length_mm
            progress = float((position - start) @ direction)
            return progress, progress
        stamp = feedback.desired.time_from_start
        desired_time = stamp.sec + stamp.nanosec / 1e9
        if self.last_desired_time is not None and desired_time < self.last_desired_time - 1e-9:
            raise ValueError('Controller trajectory time moved backwards')
        self.last_desired_time = desired_time
        position = self.planner.nozzle_position(joints)
        progress, material, interval = path.project(
            position, desired_time, self.settings.trajectory_tracking_window_mm,
        )
        nearest = path.closest_source_point(position, interval)
        self.planner.check_pose(joints, nearest, nearest)
        return progress, material

    def execute_timed(self, step):
        """Execute E-only or dwell without blocking callbacks; always finish at zero."""
        self.check_interrupted()
        speed = step.extrusion_steps_s if step.duration_s > 0 else 0.0
        if not self.joints_fresh():
            raise RuntimeError('Joint feedback is stale before timed action')
        if speed and self.speed_publisher.get_subscription_count() == 0:
            raise RuntimeError('Extruder subscriber disconnected')
        try:
            self.set_extrusion(speed, force=True)
            deadline = time.monotonic() + step.duration_s
            while time.monotonic() < deadline:
                self.check_interrupted()
                if not self.joints_fresh():
                    raise RuntimeError('Joint feedback became stale during timed action')
                if speed and self.speed_publisher.get_subscription_count() == 0:
                    raise RuntimeError('Extruder subscriber disconnected')
                rclpy.spin_once(self, timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))
        finally:
            self.set_extrusion(0.0, force=True)

    def update_print_extrusion(self, now):
        """Reject measured feed above the configured material limit before publishing."""
        feed = self.extrusion_sync.filament_speed_mm_s
        if feed > self.settings.max_extrusion_speed_mm_s:
            self.set_extrusion(0.0, force=True)
            raise RuntimeError(f'Measured filament demand {feed:.3f} mm/s exceeds the feed limit')
        super().update_print_extrusion(now)

    def run(self):
        """Parse and plan every step before enabling execution of the first motion."""
        cfg = self.settings
        parser = GCodeParser(cfg.ignored_commands, cfg.max_travel_speed_mm_s)
        with Path(cfg.file).expanduser().open() as source:
            segments = parser.parse(source)
        for command, lines in sorted(parser.ignored.items()):
            self.get_logger().warn(
                f'{command}: ignored by explicit ignored_commands configuration '
                f'({len(lines)} occurrences, first line {lines[0]}). '
                'Homing/heating/cooling is externally managed; no command or wait is performed.'
            )
        if self.config.dry_run:
            seed = np.deg2rad(self.config.dry_run_seed_deg)
        else:
            self.wait_until(self.joints_fresh, self.config.startup_timeout_s, 'fresh joint_states')
            seed = self.joints.copy()
        steps = self.planner.plan_segments(segments, seed, self.planning_progress)
        duration = sum(float(step.motion.times[-1]) if step.motion is not None else step.duration_s
                       for step in steps)
        filament = (sum(max(0, segment.delta_e) for segment in segments) *
                    cfg.extrusion_scale_factor)
        self.get_logger().info(
            f'Preflight passed: {len(segments)} source segments, {len(steps)} actions; '
            f'nominal motion/dwell time {duration:.1f} s, positive E total {filament:.3f} mm. '
            f'Look-ahead={cfg.lookahead_enabled}; compatible source segments share one action.'
        )
        self.check_interrupted()
        if self.config.dry_run:
            self.get_logger().info('DRY RUN complete: no robot or extruder commands published.')
            return
        self.wait_until(self.action.server_is_ready, self.config.startup_timeout_s,
                        'trajectory action server')
        if any(segment.delta_e != 0 for segment in segments) and cfg.extrusion_scale_factor:
            self.wait_until(lambda: self.speed_publisher.get_subscription_count() > 0,
                            self.config.startup_timeout_s, 'extruder subscriber')
        previous_sample = self.joint_received
        self.wait_until(lambda: self.joint_received != previous_sample and self.joints_fresh(),
                        self.config.startup_timeout_s, 'joint feedback after planning')
        for index, step in enumerate(steps, 1):
            line = step.segment.source_line if step.segment else 'positioning'
            if step.path is not None:
                line = f'{step.path.segments[0].source_line}-{step.path.segments[-1].source_line}'
            self.get_logger().info(f'Action {index}/{len(steps)}, source line {line}')
            self.configure_step(step)
            if step.motion is None:
                self.execute_timed(step)
            else:
                self.execute_motion(step.motion)
        self.get_logger().info('G-code completed; extrusion stopped and nozzle lifted.')


def main(args=None):
    """Deliver zero extrusion and action cancellation before shutting down ROS."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    handlers = {}
    exit_code = 0
    try:
        node = GCodeInterpreter()

        def interrupt(signum, frame):
            node.stop_requested = True

        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, interrupt)
        node.run()
    except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as error:
        exit_code = 1
        if node is None:
            print(f'G-code configuration failed: {error}')
        else:
            node.get_logger().error(str(error) or 'Interrupted')
    finally:
        try:
            if node is not None:
                node.stop()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for signum, handler in handlers.items():
                signal.signal(signum, handler)
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
