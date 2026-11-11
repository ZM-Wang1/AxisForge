"""Sampled trajectory execution, cancellation and feed-forward extrusion."""

import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectoryPoint

from extrusion_control.extrusion_calibration import MAX_STEPPER_SPEED
from print_control.motion_planner import JOINT_NAMES, duration_parts

EXTRUSION_SETTLE_STEPS = 0.5


class MotionExecutor(Node):
    """Shared action lifecycle for sampled motion and feed-forward extrusion."""

    def initialize_execution(self, config, planner):
        """Initialize execution after the concrete node has read its parameters."""
        self.config = config
        self.planner = planner
        self.stop_requested = False
        self.aborting = False
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.motion_token = None
        self.joints = None
        self.joint_received = None
        self.feedback_received = None
        self.speed = 0.0
        self.commanded_steps = 0.0
        self.last_extrusion_update = None
        self.speed_publisher = None
        self.action = None
        # Dry-run has no command publishers or action client, even on shutdown.
        if not self.config.dry_run:
            self.speed_publisher = self.create_publisher(Float32, '/stepper/speed', 1)
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
            raise RuntimeError('Printing interrupted')

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
        """Publish changes or stops; suppress nonzero output while aborting."""
        if self.speed_publisher is None:
            return
        if speed != 0 and (self.aborting or self.stop_requested):
            return
        if speed != self.speed or force:
            self.speed_publisher.publish(Float32(data=float(speed)))
            self.speed = speed

    def stream_extrusion(self, extrusion, elapsed):
        """
        Stream planned feed with gradual correction of host-clock material error.

        Correcting all error in one period oscillates when callbacks arrive two periods
        apart. Spread correction over at least four periods and twice the last interval.
        This estimates delivered steps from published commands; transport delay is unmeasured.
        """
        period = self.config.extrusion_update_period_s
        held = 0.0
        if self.last_extrusion_update is not None:
            held = elapsed - self.last_extrusion_update
            if held < period:
                return
            self.commanded_steps += self.speed * held
        position = extrusion.steps_at(elapsed)
        feed = (extrusion.steps_at(elapsed + period) - position) / period
        recovery = max(4 * period, 2 * held)
        error = position - self.commanded_steps
        speed = feed + error / recovery
        if elapsed >= extrusion.duration_s and abs(error) <= EXTRUSION_SETTLE_STEPS:
            speed = 0.0
        if not np.isfinite(speed) or abs(speed) > MAX_STEPPER_SPEED:
            self.set_extrusion(0.0, force=True)
            raise RuntimeError('Extrusion correction exceeds the firmware speed limit')
        self.set_extrusion(speed)
        self.last_extrusion_update = elapsed

    def extrusion_remaining(self, extrusion, elapsed):
        """Estimate remaining STEP pulses, including the currently held speed."""
        delivered = self.commanded_steps
        if self.last_extrusion_update is not None:
            delivered += self.speed * (elapsed - self.last_extrusion_update)
        return extrusion.steps_at(extrusion.duration_s) - delivered

    def extrusion_poll_timeout(self, started):
        """Wake for the next command deadline even after an intervening ROS callback."""
        due = started + self.last_extrusion_update + self.config.extrusion_update_period_s
        return min(0.01, max(0.0, due - time.monotonic()))

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

    def motion_feedback(self, token):
        """Use only the active trajectory's controller feedback as a liveness signal."""
        if token is self.motion_token:
            self.feedback_received = time.monotonic()

    def execute_motion(self, motion, extrusion=None):
        """Execute one phase; a planned extrusion profile runs on the trajectory clock."""
        self.check_interrupted()
        if not self.joints_fresh():
            raise RuntimeError('Joint feedback is stale before sending a trajectory')
        error = np.max(np.abs(np.rad2deg(self.joints - motion.positions[0])))
        if error > self.config.start_tolerance_deg:
            raise RuntimeError(f'Robot moved away from planned start ({error:.3f} deg)')
        self.set_extrusion(0.0, force=True)
        self.commanded_steps = 0.0
        self.last_extrusion_update = None
        self.feedback_received = None
        self.goal_handle = None
        self.result_future = None
        token = self.motion_token = object()
        self.get_logger().info(
            f'{motion.name}: {motion.times[-1]:.3f} s, {len(motion.times)} points'
        )
        self.goal_future = self.action.send_goal_async(
            self.trajectory_goal(motion),
            feedback_callback=lambda message: self.motion_feedback(token),
        )
        self.wait_until(
            self.goal_future.done, self.config.startup_timeout_s, 'trajectory acceptance',
        )
        self.goal_handle = self.goal_future.result()
        if not self.goal_handle.accepted:
            raise RuntimeError(f'{motion.name}: controller rejected trajectory')
        self.result_future = self.goal_handle.get_result_async()
        # An unstamped trajectory starts when the controller accepts it.
        started = time.monotonic()
        deadline = (started + motion.times[-1] * self.config.execution_timeout_factor +
                    self.config.stall_timeout_s)
        while not self.result_future.done():
            self.check_interrupted()
            now = time.monotonic()
            if not self.joints_fresh():
                raise RuntimeError('Joint feedback became stale')
            last_feedback = (
                self.feedback_received if self.feedback_received is not None else started
            )
            if now - last_feedback > self.config.feedback_timeout_s:
                raise RuntimeError('Controller feedback timed out')
            if now > deadline:
                raise RuntimeError(f'{motion.name}: execution timed out')
            timeout = 0.01
            if extrusion is not None:
                if self.speed_publisher.get_subscription_count() == 0:
                    raise RuntimeError('Extruder subscriber disconnected')
                self.stream_extrusion(extrusion, now - started)
                timeout = self.extrusion_poll_timeout(started)
            rclpy.spin_once(self, timeout_sec=timeout)
        self.motion_token = None
        response = self.result_future.result()
        if (response.status != GoalStatus.STATUS_SUCCEEDED or
                response.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            self.set_extrusion(0.0, force=True)
            raise RuntimeError(
                f'{motion.name}: action status={response.status}, '
                f'error={response.result.error_code}: {response.result.error_string}'
            )
        if extrusion is not None:
            # Finish only small host-clock residuals after success, with a bounded tail.
            # A failed/cancelled action above never gets this extra material.
            period = self.config.extrusion_update_period_s
            end = started + extrusion.duration_s + max(0.5, 40 * period)
            while True:
                self.check_interrupted()
                now = time.monotonic()
                elapsed = now - started
                remaining = self.extrusion_remaining(extrusion, elapsed)
                if (elapsed >= extrusion.duration_s and
                        abs(remaining) <= EXTRUSION_SETTLE_STEPS):
                    break
                if now >= end:
                    self.set_extrusion(0.0, force=True)
                    raise RuntimeError('Extrusion did not settle within the completion timeout')
                if not self.joints_fresh():
                    raise RuntimeError('Joint feedback became stale during extrusion completion')
                if self.speed_publisher.get_subscription_count() == 0:
                    raise RuntimeError('Extruder disconnected during extrusion completion')
                self.stream_extrusion(extrusion, elapsed)
                rclpy.spin_once(self, timeout_sec=self.extrusion_poll_timeout(started))
        self.set_extrusion(0.0, force=True)
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

    def stop(self):
        """Stop extrusion first; handle late acceptance and wait for cancellation."""
        self.aborting = True
        self.motion_token = None
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
