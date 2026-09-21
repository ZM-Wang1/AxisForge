#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import numpy as np
import math
import signal
import sys

JOINT_STATE_PUB_LATENCY = 2.0  # ms
STEPS = 200


class RobotLatencyTester(Node):
    def __init__(self):
        super().__init__("robot_latency_tester")

        self.traj_pub = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Mapping of joint names to velocity indices
        self.velocity_indices = {
            "shoulder_pan_joint": 5,
            "shoulder_lift_joint": 0,
            "elbow_joint": 1,
            "wrist_1_joint": 2,
            "wrist_2_joint": 3,
            "wrist_3_joint": 4,
        }

        # Home positions for each joint (degrees)
        self.home_positions = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -90.0,
            "elbow_joint": 0.0,
            "wrist_1_joint": -90.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        }

        # Oscillation ranges for each joint (degrees from home)
        self.oscillation_range = 0.5

        # Store latencies for each joint separately
        self.movement_latencies = {joint: [] for joint in self.joint_names}

        self.waiting_for_movement = False
        self.step_count = 0
        self.current_joint_index = 1
        self.current_joint_offset = 0.0
        self.direction = 1
        self.start_time = 0
        self.test_active = False

        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        self.init_robot()

    def signal_handler(self, sig, frame):
        print("\n\nCtrl+C detected! Saving data before exit...")
        self.test_active = False
        self.print_results()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def init_robot(self):
        # Move to home position with longer duration
        home_q_rad = [
            np.deg2rad(self.home_positions[joint]) for joint in self.joint_names
        ]
        self.publish_trajectory(home_q_rad, 3.0)
        time.sleep(4)  # Wait longer to ensure settling

        if self.current_joint_index == 1:
            self.duration = 0.1
            self.oscillation_range = 0.5
        elif self.current_joint_index == 2:
            self.duration = 0.006
            self.oscillation_range = 0.5
        elif self.current_joint_index > 2:
            self.duration = 0.006
            self.oscillation_range = 1.0
        self.test_active = True
        timer_interval = 2.0  # Space out commands to allow full stop
        self.timer = self.create_timer(timer_interval, self.measurement_cycle)

    def measurement_cycle(self):
        current_joint = self.joint_names[self.current_joint_index]

        # Check if we've completed all steps for all joints
        if self.current_joint_index >= len(self.joint_names):
            self.print_results()
            self.timer.cancel()
            self.test_active = False
            self.destroy_node()
            rclpy.shutdown()
            return

        # Check if we've completed steps for current joint
        if self.step_count >= STEPS:
            print(f"Completed {STEPS} measurements for {current_joint}")
            # Move to next joint
            self.current_joint_index += 1
            self.step_count = 0
            self.current_joint_offset = 0.0
            self.direction = 1
            if self.current_joint_index == 1:
                self.duration = 0.1
                self.oscillation_range = 0.5
            elif self.current_joint_index == 2:
                self.duration = 0.006
                self.oscillation_range = 0.5
            elif self.current_joint_index > 2:
                self.duration = 0.006
                self.oscillation_range = 1.0

            # Return to home position before testing next joint
            if self.current_joint_index < len(self.joint_names):
                home_q_rad = [
                    np.deg2rad(self.home_positions[joint]) for joint in self.joint_names
                ]
                self.publish_trajectory(home_q_rad, 1.0)
                time.sleep(2.5)
                print(
                    f"\nStarting measurements for {self.joint_names[self.current_joint_index]}..."
                )
            return

        # Oscillate the current joint
        if self.current_joint_offset >= self.oscillation_range:
            self.direction = -1
        elif self.current_joint_offset <= -self.oscillation_range:
            self.direction = 1

        self.current_joint_offset += self.direction * self.oscillation_range

        # Build joint positions: home for all, except current joint which oscillates
        q_deg = []
        for joint in self.joint_names:
            if joint == current_joint:
                q_deg.append(self.home_positions[joint] + self.current_joint_offset)
            else:
                q_deg.append(self.home_positions[joint])

        q_rad = np.deg2rad(q_deg).tolist()

        self.start_time = time.perf_counter()
        self.publish_trajectory(q_rad, self.duration)

        self.waiting_for_movement = True
        self.step_count += 1

    def publish_trajectory(self, positions, duration_sec):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.nanosec = int(duration_sec * 1e9)

        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)

    def joint_state_callback(self, msg):
        if self.waiting_for_movement and self.test_active and msg.velocity:
            current_joint = self.joint_names[self.current_joint_index]
            velocity_index = self.velocity_indices[current_joint]

            velocity = abs(math.degrees(msg.velocity[velocity_index]))
            if velocity >= 0.000005:
                movement_time = time.perf_counter()
                movement_latency = (movement_time - self.start_time) * 1000
                current_ros_time = self.get_clock().now()
                msg_ros_time = rclpy.time.Time.from_msg(msg.header.stamp)
                ros2_delay = (current_ros_time - msg_ros_time).nanoseconds / 1_000_000.0
                compensated_latency = (
                    movement_latency - ros2_delay - JOINT_STATE_PUB_LATENCY
                )

                if compensated_latency > 0:
                    self.movement_latencies[current_joint].append(compensated_latency)

                self.waiting_for_movement = False

    def print_results(self):
        print(f"\n{'='*70}")
        print(f"LATENCY TEST RESULTS")
        print(f"{'='*70}")

        # Write data and print summary for each joint
        for joint in self.joint_names:
            latencies = self.movement_latencies[joint]

            if latencies:
                # Write to file
                filename = f"{joint}.txt"
                try:
                    with open(filename, "w") as f:
                        for latency in latencies:
                            f.write(f"{latency:.4f}\n")
                    print(f"\n{joint}:")
                    print(f"  Data written to '{filename}'")
                except Exception as e:
                    print(f"\n{joint}:")
                    print(f"  Error writing to file: {e}")

                # Print statistics
                lat_array = np.array(latencies)
                print(f"  Average: {np.mean(lat_array):.4f} ms")
                print(f"  Min:     {np.min(lat_array):.4f} ms")
                print(f"  Max:     {np.max(lat_array):.4f} ms")
                print(f"  Std Dev: {np.std(lat_array):.4f} ms")
                print(f"  Valid:   {len(latencies)}/{STEPS}")
            else:
                print(f"\n{joint}:")
                print(f"  No data collected")

        print(f"\n{'='*70}")


def main():
    print("Starting Multi-Joint Latency Test")

    rclpy.init()
    node = RobotLatencyTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error occurred: {e}")
        node.print_results()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
