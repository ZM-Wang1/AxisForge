"""Transform and preflight G-code paths with continuous, checked robot motion."""

from dataclasses import dataclass, replace
import math

import numpy as np

from extrusion_control.extrusion_calibration import (
    MAX_MOVE_STEPS, STEPS_PER_MM, filament_speed_to_steps_s,
)
from extrusion_control.extrusion_profile import ExtrusionProfile
from print_control.bed_surface import BedSurface, subdivide
from print_control.gcode_parser import ToolpathSegment
from print_control.gcode_lookahead import ContinuousPath, build_continuous_path
from print_control.motion_planner import (
    MotionPlanner, PlannedMotion, cubic_joint_state, duration_parts,
)


@dataclass(frozen=True)
class GCodeConfig:
    """G-code settings; all lengths are mm and speeds are mm/s unless named otherwise."""

    file: str = 'cube.gcode'
    x_offset: float = -700.0
    y_offset: float = -360.0
    z_offset: float = 195.5
    origin_at_center: bool = False
    bed_size_mm: tuple = (235.0, 235.0)
    wrist_angle: float = 90.0
    nozzle_roll_deg: float = 180.0
    nozzle_pitch_deg: float = 0.0
    print_speed_multiplier: float = 1.0
    extrusion_scale_factor: float = 1.0
    first_layer_speed_factor: float = 0.4
    first_layer_height_mm: float = 0.4
    max_print_speed_mm_s: float = 100.0
    max_travel_speed_mm_s: float = 20.0
    max_extrusion_speed_mm_s: float = 45.0
    minimum_segment_duration_s: float = 0.10
    lookahead_enabled: bool = True
    max_path_acceleration_mm_s2: float = 100.0
    corner_blend_tolerance_mm: float = 0.05
    max_corner_speed_mm_s: float = 10.0
    max_continuous_points: int = 100000
    extrusion_move_margin_s: float = 0.05
    pressure_advance_s: float = 0.0
    bed_probe_file: str = ''
    bed_z_adjust_mm: float = 0.0
    bed_probe_extrapolation_mm: float = 5.0
    ignored_commands: tuple = ('G28', 'M84', 'M104', 'M105', 'M106', 'M107', 'M109',
                               'M140', 'M190')

    def validate(self):
        """Reject invalid values before loading a path or constructing ROS commands."""
        if not self.file:
            raise ValueError('file must not be empty')
        if not isinstance(self.origin_at_center, bool):
            raise ValueError('origin_at_center must be boolean')
        if not isinstance(self.lookahead_enabled, bool):
            raise ValueError('lookahead_enabled must be boolean')
        if self.max_path_acceleration_mm_s2 <= 0:
            raise ValueError('max_path_acceleration_mm_s2 must be positive')
        if self.extrusion_move_margin_s <= 0:
            raise ValueError('extrusion_move_margin_s must be positive')
        if self.pressure_advance_s < 0 or self.bed_probe_extrapolation_mm < 0:
            raise ValueError('pressure_advance_s and bed_probe_extrapolation_mm must not be '
                             'negative')
        if self.corner_blend_tolerance_mm < 0:
            raise ValueError('corner_blend_tolerance_mm must be nonnegative')
        if (type(self.max_continuous_points) is not int or self.max_continuous_points < 3):
            raise ValueError('max_continuous_points must be an integer >= 3')
        if len(self.bed_size_mm) != 2 or not all(
                math.isfinite(value) and value > 0 for value in self.bed_size_mm):
            raise ValueError('bed_size_mm must have two positive finite values')
        for name, value in vars(self).items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value):
                    raise ValueError(f'{name} must be finite')
                if name.endswith(('_factor', '_multiplier', '_mm_s', '_duration_s')):
                    if value < 0 or (value == 0 and name != 'extrusion_scale_factor'):
                        raise ValueError(f'{name} must be positive (flow may be zero)')
        if self.first_layer_height_mm < 0:
            raise ValueError('first_layer_height_mm must not be negative')
        # Motion/coordinate/extrusion semantics may never be disabled by an allowlist.
        external = {'G28', 'M18', 'M84', 'M104', 'M105', 'M106', 'M107', 'M109',
                    'M140', 'M190', 'M201', 'M203', 'M204', 'M205'}
        if set(self.ignored_commands) - external:
            raise ValueError('ignored_commands contains a code that cannot be delegated')

    def transform(self, point):
        """Preserve the old bed-origin convention, with configurable bed dimensions."""
        offset = np.array([self.x_offset, self.y_offset, self.z_offset])
        if self.origin_at_center:
            offset[:2] += np.asarray(self.bed_size_mm) / 2
        return tuple(np.asarray(point) + offset)


@dataclass
class PlannedStep:
    """A checked robot motion, firmware-counted E-only move, or zero-extrusion dwell."""

    segment: ToolpathSegment | None
    motion: PlannedMotion | None = None
    extrusion: ExtrusionProfile | None = None  # Feed-forward E of a depositing motion
    extrusion_steps_s: float = 0.0
    extrusion_steps: int = 0  # Signed STEP pulses the firmware counts for E-only moves
    duration_s: float = 0.0
    path: ContinuousPath | None = None


def smoothstep_filament(material_mm, duration_s):
    """Filament of a sampled_line: E follows its rest-to-rest 3u^2 - 2u^3 progress."""
    def filament(times):
        u = times / duration_s
        return material_mm * u**2 * (3 - 2 * u), material_mm * 6 * u * (1 - u) / duration_s
    return filament


class GCodePlanner(MotionPlanner):
    """Reuse nozzle IK and interpolation checks with joint and TCP constraints."""

    def __init__(self, settings, motion_config):
        """Use the same fixed nozzle orientation in geometry and the executor."""
        settings.validate()
        self.settings = settings
        super().__init__(replace(
            motion_config,
            nozzle_rpy_deg=(settings.nozzle_roll_deg, settings.nozzle_pitch_deg,
                            settings.wrist_angle),
        ))
        if (settings.lookahead_enabled and
                settings.corner_blend_tolerance_mm >= self.config.path_tolerance_mm):
            raise ValueError('corner_blend_tolerance_mm must be less than path_tolerance_mm')
        self.bed = None
        if settings.bed_probe_file:
            self.bed = BedSurface.load(
                settings.bed_probe_file, self.config.tcp_offset_mm, self.config.nozzle_rpy_deg,
                settings.bed_probe_extrapolation_mm,
            )

    def transform(self, point):
        """G-code to robot Base; with a probed bed, Z rides on the measured surface."""
        base = np.asarray(self.settings.transform(point), dtype=float)
        if self.bed is not None:
            base[2] = point[2] + self.bed.height(base) + self.settings.bed_z_adjust_mm
        return tuple(base)

    def nozzle_jacobian(self, joints):
        """Return the Base-frame Jacobian at the actual nozzle offset."""
        jacobian = self.robot.jacob0(joints).copy()
        offset = self.robot.fkine(joints).R @ self.offset_m
        jacobian[:3] += np.cross(jacobian[3:].T, offset).T
        return jacobian

    def extrusion_profile(self, duration_s, filament, direction):
        """Sample planned filament (mm, mm/s) at the command period; K only for deposition."""
        period = self.config.extrusion_update_period_s
        times = np.linspace(0.0, duration_s, math.ceil(duration_s / period) + 1)
        advance = self.settings.pressure_advance_s if direction > 0 else 0.0
        return ExtrusionProfile.plan(times, *filament(times), advance, direction)

    def continuous_motion(self, segments, speeds, seed, progress_callback=None):
        """Plan a full run with shared velocities and no action gaps at source corners."""
        cfg = self.settings
        path = build_continuous_path(
            segments, speeds, self.config.waypoint_spacing_mm,
            cfg.corner_blend_tolerance_mm, cfg.max_corner_speed_mm_s,
            cfg.max_path_acceleration_mm_s2, cfg.max_extrusion_speed_mm_s,
            cfg.max_continuous_points,
        )
        positions = [np.asarray(seed)]
        self.check_pose(seed, path.points[0], path.points[0])
        for index, point in enumerate(path.points[1:], 1):
            if progress_callback is not None and index % 50 == 0:
                progress_callback(index, len(path.points))
            try:
                joints = self.solve(point, positions[-1])
            except ValueError as error:
                line = segments[path.source_indices[index]].source_line
                raise ValueError(f'Continuous path source line {line}: {error}') from error
            if np.max(abs(np.rad2deg(joints - positions[-1]))) > self.config.max_joint_step_deg:
                raise ValueError(f'Line {segments[path.source_indices[index]].source_line}: '
                                 'discontinuous IK solution')
            positions.append(joints)
        positions = np.asarray(positions)
        velocities = []
        for joints, tangent, speed in zip(positions, path.tangents, path.speeds):
            try:
                velocities.append(np.linalg.solve(
                    self.nozzle_jacobian(joints), np.r_[tangent * speed / 1000, [0, 0, 0]],
                ))
            except np.linalg.LinAlgError as error:
                raise ValueError('Singular Jacobian in continuous path') from error
        motion = self.retime(PlannedMotion(
            'print' if path.direction else 'travel', positions, np.asarray(velocities),
            path.times.copy(),
        ))
        # Check actual cubic TCP dynamics, including joint-curve interpolation effects.
        scale = max(1.0, cfg.minimum_segment_duration_s / motion.times[-1])
        for index, dt in enumerate(np.diff(motion.times)):
            if progress_callback is not None and index % 50 == 0:
                progress_callback(index, len(path.points))
            for fraction in np.linspace(0, 1, 9):
                q, velocity, acceleration = cubic_joint_state(
                    *motion.positions[index:index + 2], *motion.velocities[index:index + 2],
                    dt, fraction,
                )
                point = self.nozzle_position(q)
                nearest = path.closest_source_point(point, index)
                self.check_pose(q, nearest, nearest)
                jacobian = self.nozzle_jacobian(q)[:3]
                tcp_velocity = jacobian @ velocity * 1000
                epsilon = 1e-4
                derivative = (self.nozzle_jacobian(q + epsilon * velocity)[:3] -
                              self.nozzle_jacobian(q - epsilon * velocity)[:3]) / (2 * epsilon)
                tcp_acceleration = (jacobian @ acceleration + derivative @ velocity) * 1000
                limit = max(path.speed_limits[index:index + 2])
                if limit > 0:
                    scale = max(scale, float(np.linalg.norm(tcp_velocity)) / limit)
                scale = max(scale, math.sqrt(float(np.linalg.norm(tcp_acceleration)) /
                                             cfg.max_path_acceleration_mm_s2))
        motion.times *= scale * 1.001
        motion.velocities /= scale * 1.001
        self.check_motion(motion)
        path.speeds /= motion.times[-1] / path.times[-1]
        path.times = motion.times
        duration_parts(float(motion.times[-1]))
        return motion, path

    def retime(self, motion):
        """Stretch time to satisfy extrema of the commanded joint cubic curves."""
        speed_ratio = acceleration_ratio = 1.0
        for index, dt in enumerate(np.diff(motion.times)):
            q0, q1 = motion.positions[index:index + 2]
            v0, v1 = motion.velocities[index:index + 2]
            a = 2 * q0 - 2 * q1 + dt * (v0 + v1)
            b = -3 * q0 + 3 * q1 - dt * (2 * v0 + v1)
            samples = [0.0, 1.0]
            samples.extend(-bj / (3 * aj) for aj, bj in zip(a, b)
                           if abs(aj) > 1e-12 and 0 < -bj / (3 * aj) < 1)
            for fraction in samples:
                _, velocity, acceleration = cubic_joint_state(q0, q1, v0, v1, dt, fraction)
                speed_ratio = max(speed_ratio, float(np.max(np.abs(np.rad2deg(velocity)))) /
                                  self.config.max_joint_speed_deg_s)
                acceleration_ratio = max(
                    acceleration_ratio, float(np.max(np.abs(np.rad2deg(acceleration)))) /
                    self.config.max_joint_acceleration_deg_s2,
                )
        scale = max(speed_ratio, math.sqrt(acceleration_ratio)) * 1.001
        motion.times *= scale
        motion.velocities /= scale
        duration_parts(float(motion.times[-1]))
        return motion

    def sampled_line(self, name, start, end, seed, speed):
        """Sample a rest-to-rest smoothstep line and verify interpolated geometry."""
        start, end = np.asarray(start), np.asarray(end)
        distance = float(np.linalg.norm(end - start))
        if distance <= 0:
            raise ValueError('Cannot plan a zero-length spatial segment')
        # Smoothstep peak slope is 1.5; bound spatial sample spacing accordingly.
        intervals = max(4, math.ceil(1.5 * distance / self.config.waypoint_spacing_mm))
        if intervals > 10000:
            raise ValueError(f'{name}: more than 10000 intervals')
        fraction = np.linspace(0, 1, intervals + 1)
        progress = 3 * fraction**2 - 2 * fraction**3
        xyz = start + progress[:, None] * (end - start)
        positions = [np.asarray(seed)]
        self.check_pose(seed, start, start)
        for point in xyz[1:]:
            joints = self.solve(point, positions[-1])
            if np.max(np.abs(np.rad2deg(joints - positions[-1]))) > self.config.max_joint_step_deg:
                raise ValueError(f'{name}: discontinuous IK solution')
            positions.append(joints)
        # F is a peak Cartesian speed, not the average of an accelerating segment.
        duration = max(1.5 * distance / speed, self.settings.minimum_segment_duration_s)
        times = fraction * duration
        positions = np.asarray(positions)
        velocities = np.gradient(positions, times, axis=0)
        velocities[[0, -1]] = 0.0
        motion = self.retime(PlannedMotion(name, positions, velocities, times))
        self.check_motion(motion, xyz)
        return motion

    def speed_for(self, segment):
        """Apply overrides before limits; only depositing first-layer moves slow down."""
        cfg = self.settings
        printing = segment.delta_e > 0
        limit = cfg.max_print_speed_mm_s if printing else cfg.max_travel_speed_mm_s
        speed = segment.feed_mm_s * cfg.print_speed_multiplier
        first_layer = (segment.layer == 0 if segment.layer is not None else
                       max(segment.start[2], segment.end[2]) <= cfg.first_layer_height_mm)
        if printing and first_layer:
            speed *= cfg.first_layer_speed_factor
        return min(speed, limit)

    def plan_segments(self, segments, seed, progress_callback=None):
        """Preflight the entire file, including positioning and a final nozzle lift."""
        cfg = self.settings
        spatial = [segment for segment in segments if segment.length_mm > 1e-9]
        if not spatial:
            raise ValueError('G-code must contain at least one XYZ movement')
        first = spatial[0]
        # A non-extruding initial move establishes the start, as in the old entry point.
        skip_first = first is segments[0] and first.delta_e == 0
        if self.bed is not None:
            # Straight robot lines between nearby points follow the curved bed. The skipped
            # entry move starts at the G-code origin and is never executed, so keep it whole.
            head = segments[:1] if skip_first else []
            segments = head + subdivide(segments[len(head):])
        entry = self.transform(first.end if skip_first else first.start)
        seed = np.asarray(seed, dtype=float)
        self.check_joints(seed)
        above = np.asarray(entry) + [0, 0, self.config.clearance_mm]
        above_q = self.solve(above, seed)
        approach = self.retime(PlannedMotion(
            'approach', np.array([seed, above_q]), np.zeros((2, 6)),
            np.array([0.0, self.config.approach_duration_s]),
        ))
        self.check_motion(approach)
        descent = self.sampled_line('descent', above, entry, above_q, cfg.max_travel_speed_mm_s)
        steps = [PlannedStep(None, approach), PlannedStep(None, descent)]
        joints = descent.positions[-1]
        last_point = entry
        skip_until = 0
        for index, segment in enumerate(segments):
            if index < skip_until:
                continue
            if progress_callback is not None:
                progress_callback(index, len(segments))
            if segment is first and skip_first:
                continue
            try:
                transformed = replace(segment, start=self.transform(segment.start),
                                      end=self.transform(segment.end),
                                      delta_e=segment.delta_e * cfg.extrusion_scale_factor)
                if segment.kind == 'dwell':
                    steps.append(PlannedStep(transformed, duration_s=segment.dwell_s))
                    continue
                if segment.length_mm < 1e-9:
                    feed = min(segment.feed_mm_s * cfg.print_speed_multiplier,
                               cfg.max_extrusion_speed_mm_s)
                    speed = math.copysign(filament_speed_to_steps_s(feed), transformed.delta_e)
                    if transformed.delta_e == 0:
                        speed = 0.0
                    # The firmware counts these pulses, so transport jitter cannot change
                    # the amount. round() is symmetric: equal retract/unretract cancel.
                    pulses = round(transformed.delta_e * STEPS_PER_MM)
                    if abs(pulses) > MAX_MOVE_STEPS:
                        raise ValueError(f'E-only move needs {abs(pulses)} steps; '
                                         f'firmware limit is {MAX_MOVE_STEPS}')
                    duration = abs(transformed.delta_e) / feed
                    steps.append(PlannedStep(transformed, extrusion_steps_s=speed,
                                             extrusion_steps=pulses, duration_s=duration))
                    continue
                if cfg.lookahead_enabled:
                    run = [transformed]
                    speeds = [self.speed_for(segment)]
                    skip_until = index + 1
                    while skip_until < len(segments):
                        following = segments[skip_until]
                        if (following.kind == 'dwell' or following.length_mm < 1e-9 or
                                np.sign(following.delta_e * cfg.extrusion_scale_factor) !=
                                np.sign(transformed.delta_e)):
                            break
                        run.append(replace(
                            following, start=self.transform(following.start),
                            end=self.transform(following.end),
                            delta_e=following.delta_e * cfg.extrusion_scale_factor,
                        ))
                        speeds.append(self.speed_for(following))
                        skip_until += 1

                    def report(point, count):
                        if progress_callback is not None:
                            progress_callback(index + int(point * len(run) / count), len(segments))

                    motion, path = self.continuous_motion(run, speeds, joints, report)
                    extrusion = None
                    if path.direction:
                        extrusion = self.extrusion_profile(
                            float(path.times[-1]), path.filament, path.direction,
                        )
                    steps.append(PlannedStep(transformed, motion, extrusion, path=path))
                    joints = motion.positions[-1]
                    last_point = run[-1].end
                    continue
                speed = self.speed_for(segment)
                ratio = abs(transformed.delta_e) / segment.length_mm
                if ratio:
                    speed = min(speed, cfg.max_extrusion_speed_mm_s / ratio)
                motion = self.sampled_line(
                    'print' if transformed.delta_e else 'travel', transformed.start,
                    transformed.end, joints, speed,
                )
                extrusion = None
                if ratio:
                    duration = float(motion.times[-1])
                    extrusion = self.extrusion_profile(
                        duration, smoothstep_filament(abs(transformed.delta_e), duration),
                        math.copysign(1.0, transformed.delta_e),
                    )
                steps.append(PlannedStep(transformed, motion, extrusion))
                joints = motion.positions[-1]
                last_point = transformed.end
            except ValueError as error:
                raise ValueError(f'G-code line {segment.source_line}: {error}') from error
        end = np.asarray(last_point)
        lift = self.sampled_line('lift', end, end + [0, 0, self.config.clearance_mm],
                                 joints, cfg.max_travel_speed_mm_s)
        steps.append(PlannedStep(None, lift))
        return steps
