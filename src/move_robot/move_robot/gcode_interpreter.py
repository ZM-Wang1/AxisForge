#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import math
import os
from time import sleep, time
from enum import Enum, auto
import numpy as np


class PositioningState(Enum):
    ABSOLUTE = auto()
    RELATIVE = auto()

# Reliable topics
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Best-effort topics
besteffort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

class GCodeInterpreter(Node):
    def __init__(self):
        super().__init__("gcode_interpreter")

        # Publishers
        self.pose_pub_ = self.create_publisher(Twist, "ur10e/point_pose", reliable_qos)
        self.duration_pub_ = self.create_publisher(
            Float32, f"/ur10e/movement_duration", reliable_qos
        )
        self.stepper_pub_ = self.create_publisher(Float32, "/stepper/speed", reliable_qos)

        # Subscribers
        self.movement_sub = self.create_subscription(
            Bool, "ur10e/is_moving", self.robot_moving_callback, besteffort_qos
        )
        self.toggle_log_sub = self.create_subscription(
            Bool, "/kb/toggle_log", self.toggle_log_callback, 10
        )
        self.toggle_log = True
        self.pause_sub = self.create_subscription(
            Bool, "/kb/pause", self.pause_callback, 10
        )
        self.shutdown_sub = self.create_subscription(
            Bool, "/kb/shutdown", self.shutdown_callback, 10
        )
        self.shutdown_requested = False
        self.speed_multiplier_sub = self.create_subscription(
            Float32, "/kb/speed_multiplier", self.speed_multiplier_callback, 10
        )
        self.extrusion_scale_sub = self.create_subscription(
            Float32, "/kb/extrusion_scale", self.extrusion_scale_callback, 10
        )
        self.next_step_sub = self.create_subscription(
            Bool, "/kb/next_step", self.next_step_callback, 10
        )
        self.adjust_z_offset_sub = self.create_subscription(
            Float32, "/kb/adjust_z_offset", self.adjust_z_offset_callback, 10
        )

        # Parameters
        self.declare_parameter("file", "cube.gcode")
        self.declare_parameter("x_offset", -700.0)
        self.declare_parameter("y_offset", -360.0)
        self.declare_parameter("z_offset", 195.5)
        self.declare_parameter("wrist_angle", 90.0)
        self.declare_parameter("print_speed_multiplier", 1.0)
        self.declare_parameter("extrusion_scale_factor", 1.0)
        self.declare_parameter("first_layer_speed_factor", 0.4)
        self.declare_parameter("split_threshold", 50.0)
        self.declare_parameter("origin_at_center", False)

        # Constants for stepper motor calculation
        self.SHAFT_DIAMETER = 10.0
        self.MICROSTEPPING = 32.0
        self.STEPS_PER_REVOLUTION = 200.0 * self.MICROSTEPPING
        self.STEPS_PER_MM = self.STEPS_PER_REVOLUTION / (math.pi * self.SHAFT_DIAMETER)

        # Offsets from robot base to print bed origin (in mm)
        self.X_OFFSET = (
            self.get_parameter("x_offset").get_parameter_value().double_value
        )
        self.Y_OFFSET = (
            self.get_parameter("y_offset").get_parameter_value().double_value
        )
        self.Z_OFFSET = (
            self.get_parameter("z_offset").get_parameter_value().double_value
        )
        self.WRIST_ANGLE = (
            self.get_parameter("wrist_angle").get_parameter_value().double_value
        )
        self.FIRST_LAYER_SPEED_FACTOR = (
            self.get_parameter("first_layer_speed_factor")
            .get_parameter_value()
            .double_value
        )
        self.ROBOT_MAX_SPEED = 100.0  # mm/s
        self.get_logger().info(
            f"Offsets: X={self.X_OFFSET}, Y={self.Y_OFFSET}, Z={self.Z_OFFSET}"
        )
        self.get_logger().info(f"Wrist angle: {self.WRIST_ANGLE}")
        self.EXTRUSION_SCALE_FACTOR = (
            self.get_parameter("extrusion_scale_factor")
            .get_parameter_value()
            .double_value
        )
        self.PRINT_SPEED_MULTIPLIER = (
            self.get_parameter("print_speed_multiplier")
            .get_parameter_value()
            .double_value
        )
        self.SPLIT_THRESHOLD = (
            self.get_parameter("split_threshold").get_parameter_value().double_value
        )
        origin_at_center = (
            self.get_parameter("origin_at_center").get_parameter_value().bool_value
        )

        if origin_at_center:
            self.X_OFFSET += 235.0 / 2
            self.Y_OFFSET += 235.0 / 2

        # State variables
        self.positioning_state = PositioningState.ABSOLUTE
        self.current_position = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.current_feedrate = 0.0  # mm/min
        self.current_extrusion = 0.0  # mm
        self.prev_is_moving = False
        self.is_executing = False
        self.first_move_executed = False
        self.start_time = None
        self.last_movement_time = None
        self.watchdog_timeout = 0.4  # sec
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        self.watchdog_error_count = 0

        # Time esimation
        self.total_filament = -1.0
        self.remaining_filament = -1.0
        self.extrusion_history = []
        self.history_window = 30 # seconds

        # G-code command queue and execution state
        self.gcode_commands = []
        self.command_index = 0

        # Load and parse G-code file
        self.load_gcode_file()

        self.get_logger().info("G-code Interpreter Node initialized")

    def load_gcode_file(self):
        """Load and parse the G-code file"""
        gcode_file = self.get_parameter("file").get_parameter_value().string_value

        if not os.path.exists(gcode_file):
            self.get_logger().error(f"G-code file not found: {gcode_file}")
            return

        try:
            with open(gcode_file, "r") as f:
                for line_num, line in enumerate(f, 1):
                    if line.strip().startswith(";Filament used: "):
                        try:
                            value = line.strip().split(":")[1].strip()
                            if value.endswith("m"):
                                value = value[:-1]
                            self.total_filament = float(value) * 1000.0
                            self.remaining_filament = self.total_filament
                        except ValueError:
                            self.get_logger().warn(
                                f"Invalid filament usage value on line {line_num}"
                            )
                        continue

                    parsed = self.parse_gcode_line(line)
                    if parsed:
                        parsed["line_number"] = line_num
                        self.gcode_commands.append(parsed)

            self.get_logger().info(
                f"Loaded {len(self.gcode_commands)} G-code commands from {gcode_file}"
            )

        except Exception as e:
            self.get_logger().error(f"Error loading G-code file: {str(e)}")

    def parse_gcode_line(self, line):
        """Parse a single line of G-code to extract command parameters"""
        # Remove comments and whitespace
        line = line.split(";")[0].strip()
        if not line:
            return None

        parts = line.split()
        command = {}

        for part in parts:
            if part:
                letter = part[0].upper()
                try:
                    value = float(part[1:])
                    command[letter] = value
                except (ValueError, IndexError):
                    # Handle cases with no value
                    command[letter] = None

        return command if command else None

    def robot_moving_callback(self, msg):
        """Callback for robot movement status updates"""
        curr_is_moving = msg.data

        if curr_is_moving:
            self.last_movement_time = self.get_clock().now()

        # Detect transition from moving to stopped
        if self.prev_is_moving and not curr_is_moving and self.is_executing:
            if self.toggle_log:
                self.get_logger().info(
                    "Robot has stopped moving. Executing next command."
                )
            self.execute_next_command()

        self.prev_is_moving = curr_is_moving

    def start_execution(self):
        """Start executing the G-code commands"""
        if not self.gcode_commands:
            self.get_logger().warn("No G-code commands to execute")
            return

        if self.is_executing:
            self.get_logger().warn("Execution already in progress")
            return

        self.get_logger().info("Starting G-code execution")
        self.is_executing = True
        self.command_index = 0
        self.start_time = time()

        # Execute the first command
        self.execute_next_command()

    def stop_execution(self):
        """Stop G-code execution"""
        speed_msg = Float32()
        speed_msg.data = 0.0
        self.stepper_pub_.publish(speed_msg)

        # Publish duration and home pose
        duration_msg = Float32()
        duration_msg.data = float(4.0)
        self.duration_pub_.publish(duration_msg)

        pose_msg = Twist()
        pose_msg.linear.x = self.X_OFFSET
        pose_msg.linear.y = self.Y_OFFSET
        pose_msg.linear.z = 400.0
        pose_msg.angular.x = 180.0
        pose_msg.angular.y = 0.0
        pose_msg.angular.z = self.WRIST_ANGLE
        self.pose_pub_.publish(pose_msg)

        self.get_logger().info(
            f"Watchdog exceeded {self.watchdog_error_count} times during execution."
        )

        sleep(0.25)
        self.shutdown_requested = True

    def update_extrusion_history(self, extrusion_speed_mmps):
        """Update extrusion history"""
        current_time = time()
        self.extrusion_history.append((current_time, extrusion_speed_mmps))

        # Remove old entries
        cutoff_time = current_time - self.history_window
        self.extrusion_history = [
            (t, speed) for t, speed in self.extrusion_history if t >= cutoff_time
        ]

    def get_avg_extrusion_speed(self):
        """Calculate average extrusion speed over history window"""
        if not self.extrusion_history:
            return 0.0

        # Weight recent speeds more heavily
        current_time = time()
        total_weight = 0.0
        weighted_sum = 0.0
        for t, speed in self.extrusion_history:
            age = current_time - t
            weight = max(0.1, 1.0 - (age / self.history_window))
            weighted_sum += speed * weight
            total_weight += weight

        return weighted_sum / total_weight if total_weight > 0 else 0.0

    def calculate_time_estimates(self):
        """Estimate remaining time based on extrusion history"""
        if self.total_filament <= 0 or self.remaining_filament < 0:
            return None, None, 0.0

        # Calculate progress
        filament_used = self.total_filament - self.remaining_filament
        if self.total_filament > 0:
            percentage = filament_used / self.total_filament
        else:
            percentage = 0.0

        # Calculate remaining time
        avg_speed = self.get_avg_extrusion_speed()
        if avg_speed > 0 and self.remaining_filament > 0:
            estimated_remaining_time = self.remaining_filament / avg_speed
        else:
            # Fallback: use elapsed time and progress
            if self.start_time and percentage > 0:
                elapsed_time = time() - self.start_time
                estimated_total_time = elapsed_time / percentage
                estimated_remaining_time = estimated_total_time - elapsed_time
            else:
                estimated_remaining_time = 0.0

        # Calculate total time
        if self.start_time:
            elapsed_time = time() - self.start_time
            estimated_total_time = elapsed_time + estimated_remaining_time
        else:
            estimated_total_time = estimated_remaining_time

        return estimated_remaining_time, estimated_total_time, percentage

    def execute_next_command(self):
        """Execute the next G-code command in the queue"""
        if not self.is_executing or self.command_index >= len(self.gcode_commands):
            if self.command_index >= len(self.gcode_commands):
                self.get_logger().info("G-code execution completed!")
                self.is_executing = False
                self.stop_execution()
            return

        command = self.gcode_commands[self.command_index]
        estimated_remaining_time, estimated_total_time, percentage = self.calculate_time_estimates()
        if estimated_remaining_time is not None and estimated_total_time is not None:
            remaining_minutes = int(estimated_remaining_time // 60)
            remaining_seconds = int(estimated_remaining_time % 60)
            total_minutes = int(estimated_total_time // 60)
            total_seconds = int(estimated_total_time % 60)
            time_remaining_str = f"Est. remaining: {remaining_minutes}m {remaining_seconds}s / {total_minutes}m {total_seconds}s total"
        else:
            time_remaining_str = "Estimated time remaining: calculating..."

        if self.toggle_log:
            self.get_logger().info(
                f"Executing command {self.command_index + 1}/{len(self.gcode_commands)}: {command}. Progress: {percentage:.0%}. {time_remaining_str}"
            )
        else:
            self.get_logger().info(
                f"Executing command {self.command_index + 1}/{len(self.gcode_commands)}. Progress: {percentage:.0%}. {time_remaining_str}"
            )

        # Process the command
        movement_issued = self.process_command(command)

        # Move to next command
        self.command_index += 1

        if not movement_issued and self.is_executing:
            self.execute_next_command()

    def process_command(self, command):
        """Process a single G-code command"""
        # Store previous state
        prev_position = self.current_position.copy()
        prev_extrusion = self.current_extrusion

        # Update feedrate if present
        if "F" in command and command["F"] is not None:
            self.current_feedrate = command["F"]
            self.get_logger().debug(
                f"Updated feedrate to {self.current_feedrate} mm/min"
            )

        # Handle positioning state changes
        if "G" in command:
            if command.get("G") == 90:
                self.positioning_state = PositioningState.ABSOLUTE
                self.get_logger().info("Switched to absolute positioning")
                return False
            elif command.get("G") == 91:
                self.positioning_state = PositioningState.RELATIVE
                self.get_logger().info("Switched to relative positioning")
                return False

        # Update position and extrusion
        if self.positioning_state == PositioningState.ABSOLUTE:
            if "X" in command and command["X"] is not None:
                self.current_position["X"] = command["X"]
            if "Y" in command and command["Y"] is not None:
                self.current_position["Y"] = command["Y"]
            if "Z" in command and command["Z"] is not None:
                self.current_position["Z"] = command["Z"]
            if "E" in command and command["E"] is not None:
                self.current_extrusion = command["E"]
        elif self.positioning_state == PositioningState.RELATIVE:
            if "X" in command and command["X"] is not None:
                self.current_position["X"] += command["X"]
            if "Y" in command and command["Y"] is not None:
                self.current_position["Y"] += command["Y"]
            if "Z" in command and command["Z"] is not None:
                self.current_position["Z"] += command["Z"]
            if "E" in command and command["E"] is not None:
                self.current_extrusion += command["E"]

        movement_issued = False

        # Handle different G-code commands
        if "G" in command and command.get("G") == 92:
            # Handle logical reset
            self.get_logger().info(
                f"G92: Position state reset: E={self.current_extrusion}"
            )
        elif "G" in command and command.get("G") in [0, 1]:
            movement_issued = self.execute_movement(
                prev_position, prev_extrusion, command.get("G")
            )

        return movement_issued

    def execute_movement(self, prev_position, prev_extrusion, g_code):
        """Execute a movement command (G0 or G1) and calculate its duration."""
        # Calculate deltas for both robot and extruder
        delta_x = self.current_position["X"] - prev_position["X"]
        delta_y = self.current_position["Y"] - prev_position["Y"]
        delta_z = self.current_position["Z"] - prev_position["Z"]
        delta_e = self.current_extrusion - prev_extrusion
        effective_speed_mms = 0.0

        xyz_move_distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        is_xyz_move = xyz_move_distance > 0.001
        is_e_move = abs(delta_e) > 0.001

        # Case 1: Extrusion Only (e.g. G1 F2700 E-5)
        if not is_xyz_move and is_e_move:
            self.get_logger().info(
                f"Executing extrusion-only move of {delta_e:.2f} mm."
            )

            # Use the feedrate to determine the speed of the extruder motor
            feedrate_mms = self.current_feedrate / 60.0
            if feedrate_mms > 0:
                # Calculate a duration just for this extrusion action
                duration_for_extrusion = abs(delta_e) / feedrate_mms
                extrusion_speed_mmps = delta_e / duration_for_extrusion
                stepper_speed = (
                    extrusion_speed_mmps
                    * self.steps_per_mm(extrusion_speed_mmps)
                    * self.EXTRUSION_SCALE_FACTOR
                )
            else:
                self.get_logger().error(
                    "Cannot perform extrusion-only move with zero feedrate."
                )
                stepper_speed = 0.0

            speed_msg = Float32()
            speed_msg.data = float(stepper_speed)
            self.stepper_pub_.publish(speed_msg)

            self.get_clock().sleep_for(
                rclpy.duration.Duration(nanoseconds=int(duration_for_extrusion * 1e9))
            )

            return False

        # Case 2: Movement
        elif is_xyz_move:
            duration_seconds = 0.0

            if not self.first_move_executed:
                duration_seconds = 5.0
                self.get_logger().warn(
                    f"Executing first movement, Duration={duration_seconds:.2f} s."
                )
                self.first_move_executed = True
            elif (
                xyz_move_distance > self.SPLIT_THRESHOLD
                and self.positioning_state == PositioningState.ABSOLUTE
                and g_code == 1
            ):
                # Subdivide large moves into smaller segments
                mid_command = {
                    "G": g_code,
                    "X": prev_position["X"] + delta_x / 2.0,
                    "Y": prev_position["Y"] + delta_y / 2.0,
                    "Z": prev_position["Z"] + delta_z / 2.0,
                    "E": prev_extrusion + delta_e / 2.0,
                    "F": self.current_feedrate,
                }
                final_command = {
                    "G": g_code,
                    "X": self.current_position["X"],
                    "Y": self.current_position["Y"],
                    "Z": self.current_position["Z"],
                    "E": self.current_extrusion,
                    "F": self.current_feedrate,
                }

                self.gcode_commands[self.command_index] = mid_command
                self.gcode_commands.insert(self.command_index + 1, final_command)
                self.current_position = prev_position.copy()
                self.current_extrusion = prev_extrusion

                self.command_index -= 1
                return False
            else:
                if g_code == 0:
                    effective_speed_mms = self.ROBOT_MAX_SPEED
                elif g_code == 1:
                    gcode_feedrate_mms = self.current_feedrate / 60.0
                    if gcode_feedrate_mms > 0:
                        effective_speed_mms = min(
                            gcode_feedrate_mms, self.ROBOT_MAX_SPEED
                        )
                    else:
                        effective_speed_mms = self.ROBOT_MAX_SPEED

                if effective_speed_mms > 0:
                    duration_seconds = xyz_move_distance / effective_speed_mms

            # Publish duration and pose for the robot arm
            duration_msg = Float32()
            if (
                self.positioning_state == PositioningState.ABSOLUTE
                and self.current_position["Z"] < 0.4
                and g_code == 1
            ):
                duration_msg.data = float(
                    duration_seconds
                    / (self.PRINT_SPEED_MULTIPLIER * self.FIRST_LAYER_SPEED_FACTOR)
                )
            else:
                duration_msg.data = float(
                    duration_seconds / self.PRINT_SPEED_MULTIPLIER
                )
            self.duration_pub_.publish(duration_msg)

            if self.toggle_log:
                self.get_logger().info(
                    f"Move Distance: {xyz_move_distance:.2f} mm, Duration: {duration_seconds:.2f} s"
                )
            if effective_speed_mms:
                self.get_logger().info(
                    f"Current speed: {(effective_speed_mms * self.PRINT_SPEED_MULTIPLIER * self.EXTRUSION_SCALE_FACTOR):.2f} mm/s"
                )

            pose_msg = Twist()
            pose_msg.linear.x = self.current_position["X"] + self.X_OFFSET
            pose_msg.linear.y = self.current_position["Y"] + self.Y_OFFSET
            pose_msg.linear.z = self.current_position["Z"] + self.Z_OFFSET
            pose_msg.angular.x = 180.0
            pose_msg.angular.y = 0.0
            pose_msg.angular.z = self.WRIST_ANGLE
            self.pose_pub_.publish(pose_msg)

            # Calculate and publish stepper speed for the combined move
            stepper_speed = 0.0
            if duration_seconds > 0 and delta_e > 0:
                if (
                    self.positioning_state == PositioningState.ABSOLUTE
                    and self.current_position["Z"] < 0.4
                ):
                    extrusion_speed_mmps = (
                        self.FIRST_LAYER_SPEED_FACTOR * delta_e / duration_seconds
                    )
                else:
                    extrusion_speed_mmps = delta_e / duration_seconds
                stepper_speed = (
                    extrusion_speed_mmps
                    * self.steps_per_mm(extrusion_speed_mmps)
                    * self.EXTRUSION_SCALE_FACTOR
                    * self.PRINT_SPEED_MULTIPLIER
                )

            speed_msg = Float32()
            speed_msg.data = float(stepper_speed)
            self.stepper_pub_.publish(speed_msg)

            return True

        # Case 3: G1/G0 command with no change in XYZ or E.
        else:
            self.get_logger().debug(
                "G1/G0 command with no change in position or extrusion. Skipping."
            )
            return False

    def steps_per_mm(self, extrusion_speed_mmps):
        """Return calibrated steps/mm based on extrusion speed."""
        if extrusion_speed_mmps <= 0:
            return 221.43

        # Calibration data: (estimated_speed_mmps, actual_steps_per_mm)
        speeds = np.array(
            [0.91, 1.129, 2.258, 3.387, 4.516, 5.645, 6.774, 7.9, 9.032, 10.161]
        )
        spms = np.array(
            [
                196.83,
                201.30,
                201.30,
                212.91,
                221.43,
                249.52,
                250.68,
                288.82,
                288.82,
                301.95,
            ]
        )

        return float(np.interp(extrusion_speed_mmps, speeds, spms))

    def watchdog_check(self):
        """Check if robot has stopped moving unexpectedly."""
        if self.is_executing and self.last_movement_time is not None:
            elapsed_time = (
                self.get_clock().now() - self.last_movement_time
            ).nanoseconds / 1e9
            if elapsed_time > self.watchdog_timeout:
                self.get_logger().warn(
                    f"Watchdog timeout exceeded! Executing next command."
                )
                self.watchdog_error_count += 1
                self.execute_next_command()

    def toggle_log_callback(self, msg):
        self.toggle_log = not self.toggle_log

    def speed_multiplier_callback(self, msg):
        speed_value = msg.data
        self.PRINT_SPEED_MULTIPLIER = speed_value
        self.get_logger().info(
            f"Updated print speed multiplier to {self.PRINT_SPEED_MULTIPLIER}"
        )

    def extrusion_scale_callback(self, msg):
        extrusion_value = msg.data
        self.EXTRUSION_SCALE_FACTOR = extrusion_value
        self.get_logger().info(
            f"Updated extrusion scale factor to {self.EXTRUSION_SCALE_FACTOR}"
        )

    def next_step_callback(self, msg):
        self.get_logger().info("Received next step signal. Executing next command.")
        self.execute_next_command()

    def adjust_z_offset_callback(self, msg):
        """Adjust the Z offset of the robot arm relative to current."""
        adjustment = msg.data
        if abs(adjustment) > 1.0:
            self.get_logger().warn(
                f"Z offset adjustment {adjustment:.2f} mm is too large. Must be between -1.0 and 1.0 mm."
            )
            return
        self.Z_OFFSET += adjustment

        self.get_logger().info(
            f"Adjusted z_offset by {adjustment:.2f} mm. New Z offset: {self.Z_OFFSET} mm"
        )

    def pause_callback(self, msg):
        if not msg.data or not self.is_executing:
            return

        if self.positioning_state == PositioningState.RELATIVE:
            self.get_logger().warn("Cannot pause during relative positioning mode.")
            return

        self.get_logger().info("Pause requested. Creating checkpoint.")
        self.is_executing = False

        # Stop stepper motor
        speed_msg = Float32()
        speed_msg.data = 0.0
        self.stepper_pub_.publish(speed_msg)

        # Create checkpoint
        checkpoint_path = os.path.expanduser("~/checkpoint.gcode")
        try:
            with open(checkpoint_path, "w") as f:
                f.write(f"; Checkpoint created from command {self.command_index + 1}\n")
                f.write(
                    f"; Current position: X{self.current_position['X']:.3f} Y{self.current_position['Y']:.3f} Z{self.current_position['Z']:.3f}\n"
                )
                f.write(f"; Current extrusion: {self.current_extrusion:.5f}\n")
                f.write(f"; Current feedrate: F{self.current_feedrate:.0f}\n")
                f.write(f";Filament used: {self.total_filament:.4f}m\n")

                # Set current state
                f.write("G90 ; Absolute positioning\n")
                f.write(f"F{self.current_feedrate:.0f}\n")
                f.write(f"G92 E{self.current_extrusion:.5f}\n")
                f.write(
                    f"G0 X{self.current_position['X']:.3f} Y{self.current_position['Y']:.3f} Z{self.current_position['Z']:.3f}\n"
                )

                # Write remaining commands
                for i in range(self.command_index, len(self.gcode_commands)):
                    command = self.gcode_commands[i]
                    gcode_line = self.cmd_to_gcode_string(command)
                    if gcode_line:
                        f.write(f"{gcode_line}\n")

            self.get_logger().info(f"Checkpoint saved to {checkpoint_path}")
            self.get_logger().info(
                f"Remaining commands: {len(self.gcode_commands) - self.command_index}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to create checkpoint: {str(e)}")

        self.stop_execution()

    def cmd_to_gcode_string(self, command):
        """Convert a parsed command back to G-code string"""
        if not command:
            return ""

        parts = []

        if "G" in command and command["G"] is not None:
            parts.append(f"G{int(command['G'])}")

        for param in ["X", "Y", "Z", "E", "F"]:
            if param in command and command[param] is not None:
                if param == "F":
                    parts.append(f"{param}{command[param]:.0f}")
                elif param == "E":
                    parts.append(f"{param}{command[param]:.5f}")
                else:
                    parts.append(f"{param}{command[param]:.3f}")

        return " ".join(parts) if parts else ""

    def shutdown_callback(self, msg):
        self.get_logger().info("Received shutdown signal. Exiting...")
        self.stop_execution()


def main(args=None):
    rclpy.init(args=args)
    node = GCodeInterpreter()
    node.start_execution()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
