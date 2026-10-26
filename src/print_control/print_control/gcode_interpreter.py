"""Preflight G-code toolpaths and execute sampled UR trajectories with extrusion."""

from dataclasses import fields
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32, Float32MultiArray

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
        # The nozzle orientation comes from GCodeConfig.
        config = MotionConfig(**self.read_defaults(MotionConfig(), excluded={'nozzle_rpy_deg'}))
        planner = GCodePlanner(self.settings, config)
        self.initialize_execution(planner.config, planner)
        # E-only moves are counted by the firmware, not timed by this node.
        self.move_publisher = None
        if not self.config.dry_run:
            self.move_publisher = self.create_publisher(Float32MultiArray, '/stepper/move', 1)
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

    def execute_timed(self, step):
        """Run a firmware-counted E-only move or a dwell; always finish at zero."""
        self.check_interrupted()
        moving = step.extrusion_steps != 0
        if not self.joints_fresh():
            raise RuntimeError('Joint feedback is stale before timed action')
        if moving and self.move_publisher.get_subscription_count() == 0:
            raise RuntimeError('Extruder move subscriber disconnected')
        # The firmware stops itself after the counted pulses. Also wait out transport
        # jitter, so the zero below or the next speed command cannot truncate the move.
        duration = step.duration_s + (self.settings.extrusion_move_margin_s if moving else 0.0)
        try:
            if moving:
                self.move_publisher.publish(Float32MultiArray(
                    data=[float(step.extrusion_steps), abs(step.extrusion_steps_s)],
                ))
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self.check_interrupted()
                if not self.joints_fresh():
                    raise RuntimeError('Joint feedback became stale during timed action')
                if moving and self.move_publisher.get_subscription_count() == 0:
                    raise RuntimeError('Extruder move subscriber disconnected')
                rclpy.spin_once(self, timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))
        finally:
            self.set_extrusion(0.0, force=True)

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
        if self.planner.bed is None:
            self.get_logger().warn('No bed_probe_file: Z uses the flat z_offset.')
        else:
            self.get_logger().info(
                f'Bed surface: {self.planner.bed.describe()}; bed_z_adjust_mm '
                f'{cfg.bed_z_adjust_mm:+g}. Probe again whenever the bed has been moved.'
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
        if any(step.extrusion_steps for step in steps):
            self.wait_until(lambda: self.move_publisher.get_subscription_count() > 0,
                            self.config.startup_timeout_s,
                            '/stepper/move subscriber (flash the updated DRV8825_microros)')
        previous_sample = self.joint_received
        self.wait_until(lambda: self.joint_received != previous_sample and self.joints_fresh(),
                        self.config.startup_timeout_s, 'joint feedback after planning')
        for index, step in enumerate(steps, 1):
            line = step.segment.source_line if step.segment else 'positioning'
            if step.path is not None:
                line = f'{step.path.segments[0].source_line}-{step.path.segments[-1].source_line}'
            self.get_logger().info(f'Action {index}/{len(steps)}, source line {line}')
            if step.motion is None:
                self.execute_timed(step)
            else:
                self.execute_motion(step.motion, step.extrusion)
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
