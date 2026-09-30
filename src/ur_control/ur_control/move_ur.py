import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray, Float32, String, Int8, Bool

import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import sys

MAX_UINT32 = 2**32 - 1  # 4,294,967,295
MAX_DURATION_SEC = MAX_UINT32 / 1e9

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

class URcontrol(Node):
    """
    ROS2 Node for publishing joint pose commands.

    This node subscribes to various topics to receive duration, steps, end effector rotation, joint states, and XYZ position inputs.
    It publishes joint trajectories and goal poses for controlling the robot.
    """

    def __init__(self):
        super().__init__("ur_control_node")
        self.get_logger().info("robot control with xyz node started")

        self.duration = 3.0
        self.current_xyz = np.zeros(3)

        self.is_moving = False
        self.target_q = None
        self.JOINT_TOLERANCE = 0.001
        self.ur_type = "UR16e"

        self.joint_states_received = False
        # Known-good elbow-up, tool-down pose over the bed (radians).
        # Replace these six values with the joint angles you read off the
        # pendant after jogging to a safe posture above the print area.
        self.q_seed = np.deg2rad([-3.73, -90.0, -120.0, -60.0, 90.0, 0.0])
        # Base-frame Z (metres) below which no link is allowed. The table is
        # below the robot base, so this is typically slightly negative.
        # Measure/confirm this for your mounting before trusting it.
        self.TABLE_Z_THRESHOLD = -0.02

        # Make a robot based on self.ur_type
        self.robot_DH = getattr(rtb.models.DH, self.ur_type)()
        self.robot = getattr(rtb.models, self.ur_type)()
        self.robot.base = self.robot.base
        self.robot.q = self.robot_DH.q
        self.robot_DH.base = self.robot.base
        self.robot_DH.q = self.robot.q

        # Set up subscribers
        self.duration_sub = self.create_subscription(
            Float32, f"/ur16e/movement_duration", self.get_duration, reliable_qos
        )
        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self.get_current_joint_states, besteffort_qos
        )
        self.xyz_pose_sub = self.create_subscription(
            Twist, f"/ur16e/point_pose", self.get_XYZ_pos, reliable_qos
        )
        self.joint_pose_sub = self.create_subscription(
            Float32MultiArray, f"/ur16e/target_joint_pose", self.get_joint_pos, 10
        )
        self.toggle_log_sub = self.create_subscription(
            Bool, "/kb/toggle_log", self.toggle_log_callback, 10
        )
        self.shutdown_sub = self.create_subscription(
            Bool, "/kb/shutdown", self.shutdown_callback, 10
        )

        self.toggle_log = True
        self.shutdown_requested = False
        self.shutdown_delay = 4.0  # seconds
        self.shutdown_timer = None
        self.log_timer = self.create_timer(0.005, self.send_log_data)

        # Set up publishers
        self.joint_pose_pub_ = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )
        self.goal_pub_ = self.create_publisher(
            Float32MultiArray, f"/ur16e/joint_goal", 10
        )
        self.calc_pose_pub_ = self.create_publisher(
            Float32MultiArray, f"/ur16e/calculated_xyz", 10
        )
        self.robot_info_pub_ = self.create_publisher(String, f"/ur16e/robot_log", 10)
        self.is_moving_pub_ = self.create_publisher(Bool, f"/ur16e/is_moving", 10)

        self.is_moving_timer = self.create_timer(0.002, self.publish_is_moving)

    def toggle_log_callback(self, msg):
        self.toggle_log = not self.toggle_log

    def shutdown_callback(self, msg):
        self.get_logger().info(f"Received shutdown signal. Exiting in {self.shutdown_delay} seconds.")
        self.shutdown_timer = self.create_timer(self.shutdown_delay, self.set_shutdown_flag)

    def set_shutdown_flag(self):
        self.shutdown_requested = True
        self.destroy_timer(self.shutdown_timer)

    def send_log_data(self):
        log_msg = String()
        log_msg.data = f"{self.ur_type},{str(self.current_xyz[0])},{str(self.current_xyz[1])},{str(self.current_xyz[2])},{str(self.duration)}"
        self.robot_info_pub_.publish(log_msg)

    def get_duration(self, data):
        """
        Callback function for receiving duration from "/ur16e/movement_duration" topic.
        """
        self.duration = data.data

    def get_current_joint_states(self, data):
        """
        Retrieves the current joint states from the "/joint_states" topic.
        Updates the joint angles in robot.q.

        Desired order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        Received order from /joint_states: [shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan]
                                            data.position[0], data.position[1], data.position[2], etc.
        """

        # Map by joint NAME, not by index — the driver's publish order
        # ([pan, wrist_2, wrist_3, wrist_1, elbow, lift]) does not match
        # the toolbox order ([pan, lift, elbow, wrist_1, wrist_2, wrist_3]).
        jmap = dict(zip(data.name, data.position))
        try:
            self.robot.q = np.array([
                jmap["shoulder_pan_joint"],
                jmap["shoulder_lift_joint"],
                jmap["elbow_joint"],
                jmap["wrist_1_joint"],
                jmap["wrist_2_joint"],
                jmap["wrist_3_joint"],
            ])
        except KeyError as e:
            self.get_logger().warn(f"Missing joint in /joint_states: {e}")
            return
        self.joint_states_received = True

        self.calculate_current_xyz()

        if self.is_moving and self.target_q is not None:
            # Calculate the error between current and target joint positions
            error = np.linalg.norm(self.robot.q - self.target_q)

            if error < self.JOINT_TOLERANCE:
                if self.toggle_log:
                    self.get_logger().info(
                        f"Goal reached (error: {error:.4f}). Ready for next pose."
                    )

                self.is_moving = False
                self.target_q = None

                # Publish the "ready" signal
                ready_msg = Int8()
                ready_msg.data = 1

    def calculate_current_xyz(self):
        """Publishes the current pose of the robot's end effector."""
        self.current_xyz = [pose * 1000 for pose in self.robot.fkine(self.robot.q).t]
        pose_msg = Float32MultiArray()
        pose_msg.data = self.current_xyz
        self.calc_pose_pub_.publish(pose_msg)

    def get_joint_pos(self, data):
        if self.toggle_log:
            print(
                "RECIVED:\n---\nDuration: {}s\nQ1: {}\nQ2: {}\nQ3: {}\nQ4: {}\nQ5: {}\nQ6: {}\n---".format(
                    self.duration,
                    round(data.data[0], 2),
                    round(data.data[1], 2),
                    round(data.data[2], 2),
                    round(data.data[3], 2),
                    round(data.data[4], 2),
                    round(data.data[5], 2),
                )
            )
            sys.stdout.flush()

        q = np.deg2rad(data.data)
        self.move_robot(q)

    def get_XYZ_pos(self, data):
        # Check if the target position is reachable
        if data.linear.z < 0:
            self.get_logger().info("Can't reach position: destroying node")
            self.destroy_node()
            return

        # Convert position data to meters
        xyz = [pose / 1000 for pose in [data.linear.x, data.linear.y, data.linear.z]]
        x, y, z = xyz

        # Find joint angles for the given position and orientation
        q = self.find_q_pose(x, y, z, data.angular.x, data.angular.y, data.angular.z)

        # Print received data
        if self.toggle_log:
            print(
                "RECEIVED:\n---\nDuration: {}s\nX: {}\nY: {}\nZ: {}\nRX: {}\nRY: {}\nRZ: {}\n---".format(
                    self.duration,
                    data.linear.x,
                    data.linear.y,
                    data.linear.z,
                    data.angular.x,
                    data.angular.y,
                    data.angular.z,
                )
            )
            sys.stdout.flush()

        # Move the robot to the calculated joint angles (skip if no safe solution)
        if q is None:
            self.get_logger().warn("No safe IK solution — skipping this move.")
            return
        self.move_robot(q)

    def move_robot(self, q):
        # Convert input angles to float
        q_float = [float(angle) for angle in q]
        self.target_q = np.array(q_float)
        if self.toggle_log:
            self.get_logger().info("New command received. Robot is now moving...")

        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Populate JointTrajectoryPoint
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = q_float
        joint_trajectory_point.velocities = []
        joint_trajectory_point.accelerations = []
        joint_trajectory_point.effort = []

        # Set duration
        if self.duration <= MAX_DURATION_SEC:
            joint_trajectory_point.time_from_start.nanosec = int(self.duration * 1e9)  # Convert s to ns
        else:
            joint_trajectory_point.time_from_start.sec = int(self.duration)

        # Assign trajectory point to the message
        joint_trajectory_msg.points = [joint_trajectory_point]

        # Publish joint trajectory message
        self.joint_pose_pub_.publish(joint_trajectory_msg)

        # Publish goal message
        goal_msg = Float32MultiArray()
        goal_msg.data = q_float
        self.goal_pub_.publish(goal_msg)
        self.is_moving = True

        # Print the sent robot command
        if self.toggle_log:
            print("SENT ROBOT:\n---\nDuration: {}s".format(self.duration))
            for i, angle in enumerate(q_float):
                print("Q{}: {}".format(i + 1, round(np.rad2deg(angle), 2)))
            print("---")
            sys.stdout.flush()

    def find_q_pose(self, x, y, z, rx, ry, rz):
        """
        Finds the joint pose (q) for the desired end effector XYZ position.
        Calculates the joint trajectory using inverse kinematics.
        """
        roll = np.deg2rad(rx)
        pitch = np.deg2rad(ry)
        yaw = np.deg2rad(rz)
        desiredPose = sm.SE3(x, y, z) * sm.SE3.RPY(roll, pitch, yaw, order="zyx")


        # Seed from the live robot pose only once we actually have it;
        # otherwise use the known-good elbow-up seed so the solver doesn't
        # fall into an under-table branch on the first move.
        startingQ = self.robot.q if self.joint_states_received else self.q_seed

        qp, success, _, _, _ = self.robot_DH.ik_LM(desiredPose, q0=startingQ, joint_limits=True)
        if not success:
            self.get_logger().warn("C++ solver failed to find a solution. Using Python solver instead.")
            qp = self.robot_DH.ikine_LM(desiredPose, q0=startingQ, joint_limits=True).q

        # Reject any solution where a link drops below the table.
        T_all = self.robot_DH.fkine_all(qp)
        min_z = min(float(T.t[2]) for T in T_all)
        if min_z < self.TABLE_Z_THRESHOLD:
            self.get_logger().error(
                f"IK solution dips below table (min link z={min_z:.3f} m). Refusing to move."
            )
            return None

        return qp

    def publish_is_moving(self):
        is_moving_msg = Bool()
        is_moving_msg.data = self.is_moving
        self.is_moving_pub_.publish(is_moving_msg)


def main(args=None):
    rclpy.init(args=args)
    node = URcontrol()

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
