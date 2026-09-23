"""Plan one fixed-orientation nozzle line with the repository's UR16e model."""

from dataclasses import dataclass, fields
import math

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3


JOINT_NAMES = (
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
)


@dataclass(frozen=True)
class LinePrintConfig:
    """Parameters use millimetres, degrees, seconds and motor steps/second."""

    line_length_mm: float = 30.0
    line_direction_deg: float = 0.0
    start_xyz_mm: tuple = (-660.5, -244.5, 193.62)
    nozzle_rpy_deg: tuple = (180.0, 0.0, 90.0)
    extrusion_speed_steps_s: float = 200.0
    print_speed_mm_s: float = 5.0
    travel_speed_mm_s: float = 5.0
    approach_duration_s: float = 8.0
    clearance_mm: float = 10.0
    tcp_offset_mm: tuple = (0.0, 0.0, 0.0)
    waypoint_spacing_mm: float = 1.0
    dry_run: bool = True
    dry_run_seed_deg: tuple = (-145.0, -128.0, -94.0, -47.0, 90.0, 35.0)
    joint_limits_deg: tuple = (360.0, 360.0, 180.0, 360.0, 360.0, 360.0)
    max_joint_step_deg: float = 20.0
    max_joint_speed_deg_s: float = 30.0
    max_joint_acceleration_deg_s2: float = 60.0
    min_link_z_mm: float = -20.0
    path_tolerance_mm: float = 0.2
    orientation_tolerance_deg: float = 0.2
    joint_goal_tolerance_rad: float = 0.001
    start_tolerance_deg: float = 1.0
    startup_timeout_s: float = 10.0
    feedback_timeout_s: float = 1.0
    stall_timeout_s: float = 2.0
    execution_timeout_factor: float = 3.0
    cancel_timeout_s: float = 3.0
    progress_threshold_mm: float = 0.02

    def validate(self):
        """Reject invalid values before constructing a trajectory or publisher."""
        vectors = {
            'start_xyz_mm': 3, 'nozzle_rpy_deg': 3, 'tcp_offset_mm': 3,
            'dry_run_seed_deg': 6, 'joint_limits_deg': 6,
        }
        unrestricted = {'line_direction_deg', 'min_link_z_mm'}
        for field in fields(self):
            name, value = field.name, getattr(self, field.name)
            if name == 'dry_run':
                if not isinstance(value, bool):
                    raise ValueError('dry_run must be a boolean')
                continue
            array = np.asarray(value, dtype=float)
            if not np.all(np.isfinite(array)):
                raise ValueError(f'{name} must contain finite numbers')
            if name in vectors:
                if array.shape != (vectors[name],):
                    raise ValueError(f'{name} must contain {vectors[name]} numbers')
            elif array.shape != ():
                raise ValueError(f'{name} must be a number')
            elif name == 'extrusion_speed_steps_s':
                if not 0 <= value <= 64000:
                    raise ValueError('extrusion_speed_steps_s must be in [0, 64000]')
            elif name not in unrestricted and value <= 0:
                raise ValueError(f'{name} must be positive')
        if np.any(np.asarray(self.joint_limits_deg) <= 0):
            raise ValueError('joint_limits_deg must be positive')
        if self.execution_timeout_factor < 1:
            raise ValueError('execution_timeout_factor must be >= 1')
        if self.progress_threshold_mm >= self.line_length_mm:
            raise ValueError('progress_threshold_mm must be smaller than line_length_mm')
        for distance in (self.line_length_mm, self.clearance_mm):
            if distance / self.waypoint_spacing_mm > 10000:
                raise ValueError('Path exceeds 10000 intervals; increase waypoint_spacing_mm')


@dataclass
class PlannedMotion:
    """Positions and velocities define cubic Hermite joint interpolation."""

    name: str
    positions: np.ndarray
    velocities: np.ndarray
    times: np.ndarray


def duration_parts(seconds):
    """Split a nonnegative duration into valid ROS seconds and nanoseconds."""
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError('Duration must be finite and nonnegative')
    sec, nanosec = divmod(round(seconds * 1_000_000_000), 1_000_000_000)
    if sec >= 2**31:
        raise ValueError('Duration exceeds ROS int32 seconds')
    return int(sec), int(nanosec)


def cubic_joint_state(q0, q1, v0, v1, dt, fraction):
    """Evaluate the controller's position/velocity cubic interpolation."""
    a = 2 * q0 - 2 * q1 + dt * (v0 + v1)
    b = -3 * q0 + 3 * q1 - dt * (2 * v0 + v1)
    c = dt * v0
    u = fraction
    q = ((a * u + b) * u + c) * u + q0
    velocity = (3 * a * u**2 + 2 * b * u + c) / dt
    acceleration = (6 * a * u + 2 * b) / dt**2
    return q, velocity, acceleration


class LinePrintPlanner:
    """Generate and check a complete approach, descent, print and lift."""

    def __init__(self, config):
        """Load the UR16e model and fixed tool orientation."""
        config.validate()
        self.config = config
        self.robot = rtb.models.DH.UR16e()
        # Match the installed ur_description UR16e limits, including elbow ±180°.
        for link, limit in zip(self.robot.links, np.deg2rad(config.joint_limits_deg)):
            link.qlim = [-limit, limit]
        self.rotation = SE3.RPY(config.nozzle_rpy_deg, unit='deg', order='zyx').R
        self.offset_m = np.asarray(config.tcp_offset_mm) / 1000.0
        angle = math.radians(config.line_direction_deg)
        self.direction = np.array([math.cos(angle), math.sin(angle), 0.0])
        self.start = np.asarray(config.start_xyz_mm, dtype=float)
        self.end = self.start + config.line_length_mm * self.direction

    def tool_pose(self, nozzle_xyz_mm):
        """Convert a desired nozzle tip location into a tool0 target."""
        position = np.asarray(nozzle_xyz_mm) / 1000.0 - self.rotation @ self.offset_m
        return SE3.Rt(self.rotation, position)

    def nozzle_position(self, joints):
        """Return the nozzle tip in Base coordinates (mm)."""
        pose = self.robot.fkine(joints)
        return 1000 * (pose.t + pose.R @ self.offset_m)

    def check_joints(self, joints):
        """Check joint limits and the configured link-origin height threshold."""
        if np.shape(joints) != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError('Invalid IK/joint state')
        if np.any(joints < self.robot.qlim[0]) or np.any(joints > self.robot.qlim[1]):
            raise ValueError('Joint position exceeds configured limits')
        min_z_mm = min(float(t.t[2]) for t in self.robot.fkine_all(joints)) * 1000
        if min_z_mm < self.config.min_link_z_mm:
            raise ValueError('A link origin is below min_link_z_mm')

    def check_pose(self, joints, start, end):
        """Check tip distance to a segment and fixed orientation on the model."""
        pose = self.robot.fkine(joints)
        tip = 1000 * (pose.t + pose.R @ self.offset_m)
        delta = end - start
        norm = float(delta @ delta)
        fraction = np.clip((tip - start) @ delta / norm, 0, 1) if norm else 0.0
        error = np.linalg.norm(tip - (start + fraction * delta))
        cosine = (np.trace(self.rotation.T @ pose.R) - 1) / 2
        angle = math.degrees(math.acos(np.clip(cosine, -1, 1)))
        if error > self.config.path_tolerance_mm:
            raise ValueError(f'TCP path error {error:.4f} mm exceeds path_tolerance_mm')
        if angle > self.config.orientation_tolerance_deg:
            raise ValueError(f'Orientation error {angle:.4f} deg exceeds tolerance')

    def solve(self, point, seed):
        """Solve a nozzle pose and choose joint revolutions near the seed."""
        result = self.robot.ikine_LM(self.tool_pose(point), q0=seed, joint_limits=True)
        if not result.success:
            raise ValueError(f'IK failed for nozzle XYZ {np.asarray(point).tolist()}')
        # Choose an equivalent revolution close to the previous joint solution.
        joints = seed + (result.q - seed + np.pi) % (2 * np.pi) - np.pi
        self.check_joints(joints)
        self.check_pose(joints, point, point)
        return joints

    def check_motion(self, motion, xyz=None):
        """Sample cubic geometry; also include each joint's velocity extrema."""
        cfg = self.config
        for index, dt in enumerate(np.diff(motion.times)):
            if dt <= 0 or round(dt * 1e9) < 1:
                raise ValueError('Trajectory times must increase by at least 1 ns')
            q0, q1 = motion.positions[index:index + 2]
            v0, v1 = motion.velocities[index:index + 2]
            # More samples on the initial joint-space approach than on short lines.
            count = max(9, int(np.max(np.abs(q1 - q0)) / 0.02) + 2)
            samples = list(np.linspace(0, 1, count))
            a = 2 * q0 - 2 * q1 + dt * (v0 + v1)
            b = -3 * q0 + 3 * q1 - dt * (2 * v0 + v1)
            for aj, bj in zip(a, b):
                if abs(aj) > 1e-12 and 0 < -bj / (3 * aj) < 1:
                    samples.append(-bj / (3 * aj))
            for fraction in samples:
                q, velocity, acceleration = cubic_joint_state(q0, q1, v0, v1, dt, fraction)
                self.check_joints(q)
                if np.max(np.abs(np.rad2deg(velocity))) > cfg.max_joint_speed_deg_s:
                    raise ValueError(
                        f'{motion.name}: joint speed too high; '
                        'reduce speed or increase approach_duration_s'
                    )
                if np.max(np.abs(np.rad2deg(acceleration))) > cfg.max_joint_acceleration_deg_s2:
                    raise ValueError(
                        f'{motion.name}: joint acceleration too high; '
                        'reduce speed or increase approach_duration_s'
                    )
                if xyz is not None:
                    self.check_pose(q, xyz[index], xyz[index + 1])

    def cartesian_motion(self, name, start, end, seed, speed):
        """Sample one straight segment and supply continuous joint velocities."""
        distance = np.linalg.norm(end - start)
        intervals = max(2, math.ceil(distance / self.config.waypoint_spacing_mm))
        xyz = np.linspace(start, end, intervals + 1)
        positions = [np.asarray(seed)]
        self.check_pose(seed, start, start)
        for point in xyz[1:]:
            joints = self.solve(point, positions[-1])
            if np.max(np.abs(np.rad2deg(joints - positions[-1]))) > self.config.max_joint_step_deg:
                raise ValueError(f'{name}: discontinuous IK solution')
            positions.append(joints)
        positions = np.asarray(positions)
        times = np.linspace(0, distance / speed, len(positions))
        velocities = np.gradient(positions, times, axis=0)
        velocities[[0, -1]] = 0.0
        motion = PlannedMotion(name, positions, velocities, times)
        self.check_motion(motion, xyz)
        return motion

    def plan(self, seed):
        """Plan every phase before allowing any command to be sent."""
        seed = np.asarray(seed, dtype=float)
        self.check_joints(seed)
        above = self.start + [0.0, 0.0, self.config.clearance_mm]
        above_joints = self.solve(above, seed)
        approach = PlannedMotion(
            'approach', np.array([seed, above_joints]), np.zeros((2, 6)),
            np.array([0.0, self.config.approach_duration_s]),
        )
        self.check_motion(approach)
        descent = self.cartesian_motion(
            'descent', above, self.start, above_joints, self.config.travel_speed_mm_s,
        )
        printing = self.cartesian_motion(
            'print', self.start, self.end, descent.positions[-1], self.config.print_speed_mm_s,
        )
        lift = self.cartesian_motion(
            'lift', self.end, self.end + [0.0, 0.0, self.config.clearance_mm],
            printing.positions[-1], self.config.travel_speed_mm_s,
        )
        for motion in (approach, descent, printing, lift):
            duration_parts(float(motion.times[-1]))
        return [approach, descent, printing, lift]
