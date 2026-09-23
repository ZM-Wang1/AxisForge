"""Execute a single fixed-orientation line using UR trajectory actions."""

from dataclasses import fields
import signal
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectoryPoint

from move_robot.line_print_path import (
    JOINT_NAMES, LinePrintConfig, LinePrintPlanner, duration_parts,
)


class LinePrintTest(Node):
    """Serial action execution; all extrusion decisions run in one executor."""

    def __init__(self):
        """Read parameters and create command interfaces only in execution mode."""
        super().__init__('line_print_test')
        defaults = LinePrintConfig()
        values = {}
        for field in fields(defaults):
            default = getattr(defaults, field.name)
            if isinstance(default, tuple):
                default = list(default)
            values[field.name] = self.declare_parameter(field.name, default).value
        self.config = LinePrintConfig(**values)
        self.config.validate()
        self.planner = LinePrintPlanner(self.config)
        self.stop_requested = False
        self.aborting = False
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.joints = None
        self.joint_received = None
        self.feedback_received = None
        self.feedback_error = None
        self.print_progress = 0.0
        self.progress_received = None
        self.at_print_end = False
        self.speed = 0.0
        self.speed_publisher = None
        self.action = None
        # Dry-run has no command publishers or action client, even on shutdown.
        if not self.config.dry_run:
            self.speed_publisher = self.create_publisher(Float32, '/stepper/speed', 10)
            self.action = ActionClient(
                self, FollowJointTrajectory,
                '/scaled_joint_trajectory_controller/follow_joint_trajectory',
            )
            self.create_subscription(
                JointState, '/joint_states', self.on_joints, qos_profile_sensor_data,
            )

    def on_joints(self, message):
        """Accept fresh, finite joint states and order them by joint name."""
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp == 0 or abs(age) > self.config.feedback_timeout_s:
            return
        mapping = dict(zip(message.name, message.position))
        if not all(name in mapping for name in JOINT_NAMES):
            return
        joints = np.array([mapping[name] for name in JOINT_NAMES])
        if np.all(np.isfinite(joints)):
            self.joints = joints
            self.joint_received = time.monotonic()

    def joints_fresh(self):
        """Report whether a recent valid joint sample is available."""
        return (self.joint_received is not None and
                time.monotonic() - self.joint_received < self.config.feedback_timeout_s)

    def check_interrupted(self):
        """Unwind execution while keeping ROS alive for stop and cancellation."""
        if self.stop_requested or not rclpy.ok():
            raise RuntimeError('Test interrupted')

    def wait_until(self, predicate, timeout, description):
        """Process callbacks until a condition, interruption or deadline."""
        deadline = time.monotonic() + timeout
        while True:
            self.check_interrupted()
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Timeout: {description}')
            rclpy.spin_once(self, timeout_sec=0.02)

    def set_extrusion(self, speed, force=False):
        """Publish changes or explicit stop commands; suppress positive abort output."""
        if self.speed_publisher is None:
            return
        if speed > 0 and (self.aborting or self.stop_requested):
            return
        if speed != self.speed or force:
            self.speed_publisher.publish(Float32(data=float(speed)))
            self.speed = speed

    def on_feedback(self, message, printing):
        """Track actual nozzle progress during the printing action."""
        feedback = message.feedback
        self.feedback_received = time.monotonic()
        if not printing or self.aborting:
            return
        mapping = dict(zip(feedback.joint_names, feedback.actual.positions))
        if not all(name in mapping for name in JOINT_NAMES):
            self.feedback_error = 'Controller feedback is missing joint positions'
            return
        joints = np.array([mapping[name] for name in JOINT_NAMES])
        if not np.all(np.isfinite(joints)):
            self.feedback_error = 'Controller feedback contains nonfinite positions'
            return
        position = self.planner.nozzle_position(joints)
        progress = float((position - self.planner.start) @ self.planner.direction)
        if progress > self.print_progress + self.config.progress_threshold_mm:
            self.print_progress = progress
            self.progress_received = self.feedback_received
        if progress >= self.config.line_length_mm - self.config.progress_threshold_mm:
            self.at_print_end = True

    def trajectory_goal(self, motion):
        """Encode joint samples and final tolerances as an action goal."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for joints, velocities, seconds in zip(motion.positions, motion.velocities, motion.times):
            point = JointTrajectoryPoint()
            point.positions = joints.tolist()
            point.velocities = velocities.tolist()
            sec, nanosec = duration_parts(float(seconds))
            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec
            goal.trajectory.points.append(point)
        goal.goal_tolerance = [
            JointTolerance(name=name, position=self.config.joint_goal_tolerance_rad)
            for name in JOINT_NAMES
        ]
        sec, nanosec = duration_parts(self.config.stall_timeout_s)
        goal.goal_time_tolerance.sec = sec
        goal.goal_time_tolerance.nanosec = nanosec
        return goal

    def execute_motion(self, motion):
        """Execute one phase, enabling extrusion only after actual print progress."""
        self.check_interrupted()
        if not self.joints_fresh():
            raise RuntimeError('Joint feedback is stale before sending a trajectory')
        error = np.max(np.abs(np.rad2deg(self.joints - motion.positions[0])))
        if error > self.config.start_tolerance_deg:
            raise RuntimeError(f'Robot moved away from planned start ({error:.3f} deg)')
        self.set_extrusion(0.0, force=True)
        self.feedback_received = None
        self.feedback_error = None
        self.print_progress = 0.0
        self.progress_received = None
        self.at_print_end = False
        self.goal_handle = None
        self.result_future = None
        printing = motion.name == 'print'
        self.get_logger().info(
            f'{motion.name}: {motion.times[-1]:.3f} s, {len(motion.times)} points'
        )
        self.goal_future = self.action.send_goal_async(
            self.trajectory_goal(motion),
            feedback_callback=lambda msg: self.on_feedback(msg, printing),
        )
        self.wait_until(
            self.goal_future.done, self.config.startup_timeout_s, 'trajectory acceptance',
        )
        self.goal_handle = self.goal_future.result()
        if not self.goal_handle.accepted:
            raise RuntimeError(f'{motion.name}: controller rejected trajectory')
        self.result_future = self.goal_handle.get_result_async()
        started = time.monotonic()
        deadline = (started + motion.times[-1] * self.config.execution_timeout_factor +
                    self.config.stall_timeout_s)
        while not self.result_future.done():
            self.check_interrupted()
            now = time.monotonic()
            if self.feedback_error:
                raise RuntimeError(self.feedback_error)
            if not self.joints_fresh():
                raise RuntimeError('Joint feedback became stale')
            last_feedback = (
                self.feedback_received if self.feedback_received is not None else started
            )
            if now - last_feedback > self.config.feedback_timeout_s:
                raise RuntimeError('Controller feedback timed out')
            if now > deadline:
                raise RuntimeError(f'{motion.name}: execution timed out')
            if printing:
                last_progress = (
                    self.progress_received if self.progress_received is not None else started
                )
                if not self.at_print_end and now - last_progress > self.config.stall_timeout_s:
                    raise RuntimeError('Print motion stalled; extrusion stopped')
                if (self.config.extrusion_speed_steps_s > 0 and
                        self.speed_publisher.get_subscription_count() == 0):
                    raise RuntimeError('Extruder subscriber disconnected')
                if self.progress_received is not None and not self.at_print_end:
                    self.set_extrusion(self.config.extrusion_speed_steps_s)
                else:
                    self.set_extrusion(0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.set_extrusion(0.0, force=True)
        response = self.result_future.result()
        if (response.status != GoalStatus.STATUS_SUCCEEDED or
                response.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError(
                f'{motion.name}: action status={response.status}, '
                f'error={response.result.error_code}: {response.result.error_string}'
            )
        # Require a new joint sample before checking the end of this phase.
        previous_sample = self.joint_received
        self.wait_until(
            lambda: self.joint_received != previous_sample and self.joints_fresh(),
            self.config.feedback_timeout_s, 'joint feedback after completion',
        )
        error = np.max(np.abs(self.joints - motion.positions[-1]))
        if error > self.config.joint_goal_tolerance_rad:
            raise RuntimeError(
                f'{motion.name}: measured end joints differ from the goal ({error:.6f} rad)'
            )
        if printing and self.progress_received is None:
            raise RuntimeError(
                'Print completed without observed progress; check controller feedback'
            )

    def run(self):
        """Plan all phases, then execute each successful phase in sequence."""
        cfg = self.config
        if cfg.dry_run:
            seed = np.deg2rad(cfg.dry_run_seed_deg)
            self.get_logger().info(
                'DRY RUN: using configured reference joints; no commands will be published.'
            )
        else:
            self.wait_until(self.joints_fresh, cfg.startup_timeout_s, 'fresh joint_states')
            seed = self.joints.copy()
        motions = self.planner.plan(seed)
        self.get_logger().info(
            f'Nozzle XYZ mm: {self.planner.start.tolist()} -> {self.planner.end.tolist()}; '
            f'RPY deg={list(cfg.nozzle_rpy_deg)}, extrusion={cfg.extrusion_speed_steps_s} steps/s'
        )
        self.get_logger().info(
            'Planned phases: ' + ', '.join(f'{m.name}={m.times[-1]:.3f}s' for m in motions)
        )
        self.check_interrupted()
        if cfg.dry_run:
            return
        self.wait_until(
            self.action.server_is_ready, cfg.startup_timeout_s, 'trajectory action server',
        )
        if cfg.extrusion_speed_steps_s > 0:
            self.wait_until(
                lambda: self.speed_publisher.get_subscription_count() > 0,
                cfg.startup_timeout_s, 'extruder subscriber on /stepper/speed',
            )
        # Process queued states after planning; stale start poses must not be sent.
        previous_sample = self.joint_received
        self.wait_until(
            lambda: self.joint_received != previous_sample and self.joints_fresh(),
            cfg.startup_timeout_s, 'joint feedback after planning',
        )
        for motion in motions:
            self.execute_motion(motion)
        self.get_logger().info('Line print completed; extrusion stopped and nozzle lifted.')

    def stop(self):
        """Stop extrusion first; handle late acceptance and wait for cancellation."""
        self.aborting = True
        self.set_extrusion(0.0, force=True)
        if self.config.dry_run or not rclpy.ok():
            return
        deadline = time.monotonic() + self.config.cancel_timeout_s
        # A timed-out send request may still be accepted: keep servicing its future.
        if self.goal_future is not None and self.goal_handle is None:
            while not self.goal_future.done() and time.monotonic() < deadline:
                self.set_extrusion(0.0, force=True)
                rclpy.spin_once(self, timeout_sec=0.02)
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
            else:
                self.get_logger().error(
                    'Acceptance is still unresolved: robot stop cannot be confirmed. '
                    'Use pendant stop.'
                )
        if self.goal_handle is not None and self.goal_handle.accepted:
            if self.result_future is None:
                self.result_future = self.goal_handle.get_result_async()
            if not self.result_future.done():
                cancel = self.goal_handle.cancel_goal_async()
                while time.monotonic() < deadline and not self.result_future.done():
                    self.set_extrusion(0.0, force=True)
                    rclpy.spin_once(self, timeout_sec=0.02)
                if not self.result_future.done():
                    self.get_logger().error(
                        'Trajectory termination not confirmed. Use pendant stop.'
                    )
                elif cancel.done() and not cancel.result().goals_canceling:
                    self.get_logger().info(
                        'Trajectory ended before cancellation was acknowledged.'
                    )
        # Give reliable DDS a short bounded opportunity to deliver the stop command.
        for _ in range(5):
            self.set_extrusion(0.0, force=True)
            rclpy.spin_once(self, timeout_sec=0.02)


def main(args=None):
    """Run once and deliver stop/cancel before shutting down ROS."""
    # Keep ROS alive while handling Ctrl+C/SIGTERM so stop/cancel can be delivered.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    exit_code = 0
    previous_handlers = {}
    try:
        node = LinePrintTest()

        def request_stop(signum, frame):
            node.stop_requested = True

        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_stop)
        node.run()
    except (ValueError, RuntimeError, KeyboardInterrupt) as error:
        exit_code = 1
        if node is not None:
            node.get_logger().error(str(error) or 'Interrupted')
        else:
            print(f'Line print configuration failed: {error}')
    finally:
        try:
            if node is not None:
                node.stop()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
