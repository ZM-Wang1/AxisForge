"""
Jog the nozzle onto a 3x3 bed grid from the keyboard and record the touch points.

Every move uses an upright nozzle (roll 180, pitch 0) with the printing yaw and
TCP, so the Base-frame bed heights can also be used for tilted printing.
"""

import argparse
from dataclasses import fields, replace
from datetime import datetime
import os
from pathlib import Path
import select
import signal
import sys
import termios
import tty

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
import yaml

from print_control.gcode_path import GCodeConfig, GCodePlanner
from print_control.motion_executor import MotionExecutor
from print_control.motion_planner import MotionConfig, PlannedMotion

STEPS_MM = {'1': 1.0, '2': 0.1, '3': 0.05, '4': 0.01}
JOG_KEYS = {
    'w': (0, 0, 1), 's': (0, 0, -1),
    'i': (0, 1, 0), 'k': (0, -1, 0), 'j': (-1, 0, 0), 'l': (1, 0, 0),
}
HELP = """
Bed probe: put paper under the nozzle and lower until it just drags.
  w / s        Z up / down (one Z step)
  i / k        +Y / -Y      (G-code bed axes, one XY step, default 10 mm)
  j / l        -X / +X
  1 2 3 4      Z step 1 / 0.1 / 0.05 / 0.01 mm
  space        record this point (saved immediately)
  n / p        next / previous point (lifts to the hover height first)
  h            lift to the hover height here
  q            lift, save and quit          Ctrl-C  stop the robot and quit
  ?            help
"""


def grid_points(bed_size_mm, margin_mm):
    """Return 3x3 G-code XY points in serpentine order, starting at the front left."""
    columns = [(margin_mm, size / 2, size - margin_mm) for size in bed_size_mm]
    points = []
    for row, y in enumerate(columns[1]):
        xs = columns[0] if row % 2 == 0 else columns[0][::-1]
        points.extend((x, y) for x in xs)
    return points


def load_configs(path, dry_run):
    """Read the gcode_interpreter section so probing and printing share one model."""
    params = yaml.safe_load(Path(path).read_text())['gcode_interpreter']['ros__parameters']

    def build(cls, excluded=()):
        names = {field.name for field in fields(cls)} - set(excluded)
        return cls(**{key: tuple(value) if isinstance(value, list) else value
                      for key, value in params.items() if key in names})

    settings = build(GCodeConfig)
    motion = build(MotionConfig, {'nozzle_rpy_deg'})
    return settings, replace(motion, dry_run=dry_run)


def rounded(values, digits=4):
    """Plain YAML floats."""
    return [round(float(value), digits) for value in values]


class BedProbe(MotionExecutor):
    """Keyboard jogging with an upright nozzle; records nozzle touch points."""

    def __init__(self, settings, motion_config, args):
        """Plan with the printing planner; dry runs simulate the robot."""
        super().__init__('bed_probe')
        if args.margin_mm <= 0 or 2 * args.margin_mm >= min(settings.bed_size_mm):
            raise ValueError('margin_mm must leave a positive grid inside bed_size_mm')
        if (args.paper_mm < 0 or args.floor_mm <= 0 or args.jog_speed <= 0 or
                args.xy_step_mm <= 0):
            raise ValueError('paper_mm must be >= 0; floor_mm, jog_speed and xy_step_mm > 0')
        # Write where gcode_interpreter reads; never plan on the surface being replaced.
        self.output = Path(args.output or settings.bed_probe_file or 'bed_probe.yaml')
        self.output = self.output.expanduser().resolve()
        self.settings = replace(settings, bed_probe_file='', nozzle_roll_deg=180.0,
                                nozzle_pitch_deg=0.0)
        self.args = args
        planner = GCodePlanner(self.settings, motion_config)
        self.initialize_execution(planner.config, planner)
        self.points = grid_points(settings.bed_size_mm, args.margin_mm)
        self.records = {}
        self.index = 0
        self.step_mm = 0.1
        self.target = None
        self.seed = None
        if self.config.dry_run:
            self.joints = np.deg2rad(self.config.dry_run_seed_deg)

    # --- geometry -------------------------------------------------------------------------

    def bed_point(self, xy, height=0.0):
        """Robot Base position of a G-code point at a height above the configured bed."""
        return np.asarray(self.settings.transform((xy[0], xy[1], height)))

    def gcode_xy(self, base):
        """Invert the XY translation of GCodeConfig.transform."""
        origin = self.bed_point((0.0, 0.0))
        return np.asarray(base)[:2] - origin[:2]

    def hover(self, xy):
        """Start every point clearance_mm above the configured bed height."""
        return self.bed_point(xy, self.config.clearance_mm)

    # --- motion ---------------------------------------------------------------------------

    def run_motion(self, motion):
        """Execute on the robot, or adopt the planned end in a dry run."""
        if self.config.dry_run:
            self.joints = motion.positions[-1].copy()
        else:
            # Planning a long travel line takes longer than feedback_timeout_s without
            # spinning; take a new joint sample so execute_motion checks the real start.
            self.wait_until(self.joints_fresh, self.config.feedback_timeout_s,
                            'fresh joint_states after planning')
            self.execute_motion(motion)
        self.seed = motion.positions[-1].copy()

    def approach(self, end):
        """Joint-space move from the current pose to the first hover point."""
        if not self.config.dry_run:
            self.wait_until(self.joints_fresh, self.config.startup_timeout_s, 'fresh joint_states')
        seed = self.joints.copy()
        self.planner.check_joints(seed)
        goal = self.planner.solve(end, seed)
        motion = self.planner.retime(PlannedMotion(
            'approach', np.array([seed, goal]), np.zeros((2, 6)),
            np.array([0.0, self.config.approach_duration_s]),
        ))
        self.planner.check_motion(motion)
        self.run_motion(motion)
        self.target = np.asarray(end, dtype=float)

    def line_to(self, end, name, speed):
        """Straight fixed-orientation move from the current target."""
        end = np.asarray(end, dtype=float)
        if np.linalg.norm(end - self.target) < 1e-9:
            return
        motion = self.planner.sampled_line(name, self.target, end, self.seed, speed)
        self.run_motion(motion)
        self.target = end

    def lift(self):
        """Raise to the hover height without changing XY."""
        hover_z = self.hover(self.points[self.index])[2]
        if self.target[2] < hover_z:
            self.line_to([*self.target[:2], hover_z], 'lift', self.settings.max_travel_speed_mm_s)

    def go_to(self, index):
        """Lift, then travel at the hover height to the selected grid point."""
        self.lift()
        self.index = index
        self.line_to(self.hover(self.points[index]), 'travel', self.settings.max_travel_speed_mm_s)

    def jog(self, direction):
        """Z by the selected step, X/Y by xy_step_mm; never below floor_mm under the bed."""
        direction = np.asarray(direction, dtype=float)
        lateral = direction[2] == 0
        end = self.target + (self.args.xy_step_mm if lateral else self.step_mm) * direction
        floor = self.bed_point(self.points[self.index])[2] - self.args.floor_mm
        if end[2] < floor - 1e-9:
            raise ValueError(f'{self.args.floor_mm:g} mm below the configured bed height '
                             'is the jog limit; check z_offset or paper position')
        speed = self.settings.max_travel_speed_mm_s if lateral else self.args.jog_speed
        self.line_to(end, 'jog', speed)

    # --- recording ------------------------------------------------------------------------

    def nozzle_now(self):
        """Measured nozzle tip from a joint sample taken after the robot stopped."""
        if not self.config.dry_run:
            previous = self.joint_received
            self.wait_until(lambda: self.joint_received != previous and self.joints_fresh(),
                            self.config.feedback_timeout_s, 'joint sample at the touch point')
        return self.planner.nozzle_position(self.joints), self.joints.copy()

    def record(self):
        """Store the touch point and rewrite the output file."""
        nozzle, joints = self.nozzle_now()
        self.records[self.index] = {
            'point': self.index + 1,
            'grid_xy': rounded(self.points[self.index], 3),
            'gcode_xy': rounded(self.gcode_xy(nozzle)),
            'nozzle_base_mm': rounded(nozzle),
            'bed_z_mm': round(float(nozzle[2]) - self.args.paper_mm, 4),
            'commanded_base_mm': rounded(self.target),
            'joints_rad': rounded(joints, 6),
        }
        self.save()
        print(f'Recorded point {self.index + 1}: bed Z {self.records[self.index]["bed_z_mm"]:.3f}'
              f' mm (nozzle {nozzle[2]:.3f} - paper {self.args.paper_mm:g}) -> {self.output}')

    def save(self):
        """Write atomically so an interrupted session keeps every recorded point."""
        cfg = self.settings
        data = {
            'description': 'Nozzle tip touching the bed through paper; robot Base frame, mm',
            'stamp': datetime.now().isoformat(timespec='seconds'),
            'config': str(Path(self.args.config).resolve()),
            'paper_thickness_mm': self.args.paper_mm,
            'nozzle_rpy_deg': rounded(self.planner.config.nozzle_rpy_deg, 3),
            'tcp_offset_mm': rounded(self.planner.config.tcp_offset_mm, 3),
            'transform': {
                'origin_at_center': cfg.origin_at_center,
                'bed_size_mm': rounded(cfg.bed_size_mm, 3),
                'x_offset': cfg.x_offset, 'y_offset': cfg.y_offset, 'z_offset': cfg.z_offset,
            },
            'points': [self.records[index] for index in sorted(self.records)],
        }
        temporary = self.output.with_name(self.output.name + '.tmp')
        temporary.write_text(yaml.safe_dump(data, sort_keys=False))
        temporary.replace(self.output)

    # --- keyboard -------------------------------------------------------------------------

    def on_key(self, key):
        """Apply one key; return False to quit. Planning errors leave the robot in place."""
        if key in STEPS_MM:
            self.step_mm = STEPS_MM[key]
        elif key in JOG_KEYS:
            self.jog(JOG_KEYS[key])
        elif key == ' ':
            self.record()
        elif key in ('n', 'p'):
            self.go_to((self.index + (1 if key == 'n' else -1)) % len(self.points))
        elif key == 'h':
            self.lift()
        elif key == '?':
            print(HELP)
        elif key == 'q':
            self.lift()
            if self.records:
                self.save()
            return False
        return True

    def status(self):
        """One line: point, position relative to the configured bed, step and progress."""
        x, y = self.points[self.index]
        above = self.target[2] - self.bed_point(self.points[self.index])[2]
        mark = '*' if self.index in self.records else ' '
        return (f'[{self.index + 1}/9{mark}] grid X{x:g} Y{y:g} | nozzle Base '
                f'({self.target[0]:.2f}, {self.target[1]:.2f}, {self.target[2]:.3f}) | '
                f'{above:+.3f} mm vs z_offset | step Z {self.step_mm:g} / '
                f'XY {self.args.xy_step_mm:g} mm | '
                f'recorded {len(self.records)}/9')

    def read_key(self, stream, timeout):
        """Return one character, or None; keep ROS callbacks alive while waiting."""
        if not self.config.dry_run:
            rclpy.spin_once(self, timeout_sec=0.0)
        ready, _, _ = select.select([stream], [], [], timeout)
        if not ready:
            return None
        key = os.read(stream.fileno(), 1).decode(errors='ignore')
        if key == '\x1b':
            os.read(stream.fileno(), 8)  # Discard the rest of an escape sequence.
            return None
        return key

    def run(self, stream=sys.stdin):
        """Approach point 1, then process keys until q or an interruption."""
        print(HELP)
        print(f'Output: {self.output}')
        if not self.config.dry_run:
            self.wait_until(self.action.server_is_ready, self.config.startup_timeout_s,
                            'trajectory action server')
        self.approach(self.hover(self.points[0]))
        print(self.status())
        descriptor = stream.fileno()
        terminal = termios.tcgetattr(descriptor)
        tty.setcbreak(descriptor)  # Single keys; Ctrl-C still raises SIGINT.
        try:
            while True:
                self.check_interrupted()
                key = self.read_key(stream, 0.02)
                if key is None:
                    continue
                try:
                    if not self.on_key(key):
                        break
                except ValueError as error:
                    print(f'Rejected (robot not moved): {error}')
                # Keys pressed during a move must not queue further moves.
                termios.tcflush(descriptor, termios.TCIFLUSH)
                print(self.status())
        finally:
            termios.tcsetattr(descriptor, termios.TCSADRAIN, terminal)
        missing = [str(index + 1) for index in range(len(self.points))
                   if index not in self.records]
        if missing:
            print(f'Not recorded: points {", ".join(missing)}')


def parse_args(argv):
    """Probe options; the robot model and offsets come from the interpreter YAML."""
    default_config = Path(get_package_share_directory('axisforge_bringup'),
                          'config', 'ur_master.yaml')
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=str(default_config),
                        help='YAML with the gcode_interpreter parameters')
    parser.add_argument('--output', help='result YAML (default: bed_probe_file from --config)')
    parser.add_argument('--margin-mm', dest='margin_mm', type=float, default=30.0,
                        help='grid distance from the bed edges')
    parser.add_argument('--paper-mm', dest='paper_mm', type=float, default=0.1,
                        help='paper thickness subtracted from the nozzle Z')
    parser.add_argument('--floor-mm', dest='floor_mm', type=float, default=3.0,
                        help='deepest jog below the configured bed height (z_offset)')
    parser.add_argument('--jog-speed', dest='jog_speed', type=float, default=5.0,
                        help='peak speed of Z jog steps, mm/s')
    parser.add_argument('--xy-step-mm', dest='xy_step_mm', type=float, default=10.0,
                        help='X/Y jog step; moves at max_travel_speed_mm_s')
    parser.add_argument('--dry-run', action='store_true', help='simulate; no robot commands')
    return parser.parse_args(argv)


def main(argv=None):
    """Cancel any active trajectory before shutting down ROS."""
    args = parse_args(remove_ros_args(sys.argv if argv is None else argv)[1:])
    settings, motion = load_configs(args.config, args.dry_run)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    handlers = {}
    exit_code = 0
    try:
        node = BedProbe(settings, motion, args)

        def interrupt(signum, frame):
            node.stop_requested = True

        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, interrupt)
        node.run()
    except (OSError, ValueError, RuntimeError) as error:
        exit_code = 1
        print(f'Bed probe stopped: {error}')
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
