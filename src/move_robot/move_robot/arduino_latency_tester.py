#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import numpy as np


class ArduinoLatencyTester(Node):
    def __init__(self):
        super().__init__("arduino_latency_tester")
        self.publisher = self.create_publisher(Float32, "stepper/speed", 10)
        self.subscription = self.create_subscription(
            Float32, "stepper/pong", self.pong_callback, 10
        )
        self.ping_time = 0
        self.latencies = []
        self.test_count = 0

    def pong_callback(self, msg):
        pong_time = time.perf_counter_ns()
        latency = (pong_time - self.ping_time) / 2
        self.latencies.append(latency)

    def ping(self):
        self.ping_time = time.perf_counter_ns()
        msg = Float32()
        msg.data = float(self.test_count)
        self.publisher.publish(msg)
        self.test_count += 1

def main():
    rclpy.init()
    tester = ArduinoLatencyTester()

    total_tests = 1000
    timeouts = 0

    for i in range(total_tests):
        tester.ping()

        timeout = time.time() + 0.95
        while len(tester.latencies) < tester.test_count and time.time() < timeout:
            rclpy.spin_once(tester, timeout_sec=0.001)

        if len(tester.latencies) < tester.test_count:
            print(f"Test {i+1} timed out")
            timeouts += 1

        time.sleep(0.05)

    rclpy.shutdown()

    if tester.latencies:
        # Convert to milliseconds
        latencies_ms = [lat / 1e6 for lat in tester.latencies]

        # Write to file
        try:
            with open("arduino_latencies.txt", "w") as f:
                for latency in latencies_ms:
                    f.write(f"{latency:.4f}\n")
            print(f"Latency data written to 'arduino_latencies.txt'")
        except Exception as e:
            print(f"Error writing to file: {e}")

        # Calculate statistics
        lat_array = np.array(latencies_ms)
        avg_latency = np.mean(lat_array)
        min_latency = np.min(lat_array)
        max_latency = np.max(lat_array)
        std_dev = np.std(lat_array)

        print(f"\n{'='*50}")
        print(f"ARDUINO LATENCY TEST RESULTS")
        print(f"{'='*50}")
        print(f"Average one-way latency: {avg_latency:.4f} ms")
        print(f"Min:                     {min_latency:.4f} ms")
        print(f"Max:                     {max_latency:.4f} ms")
        print(f"Std Dev:                 {std_dev:.4f} ms")
        print(f"Tests completed:         {len(tester.latencies)}/{total_tests}")
        print(f"Timeouts:                {timeouts}")
        print(f"{'='*50}")


if __name__ == "__main__":
    main()
