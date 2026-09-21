import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import math

# --- Constants ---
SHAFT_DIAMETER = 10.0
MICROSTEPPING = 32.0
STEPS_PER_REVOLUTION = 200.0 * MICROSTEPPING
# This is the theoretical value we are calibrating against
THEORETICAL_STEPS_PER_MM = 221.43 #STEPS_PER_REVOLUTION / (math.pi * SHAFT_DIAMETER)

class ExtruderCalibrator(Node):

    def __init__(self):
        super().__init__('extruder_calibrator')
        
        # Publisher to the /stepper_speed topic 
        self.publisher_ = self.create_publisher(Float32, 'stepper/speed', 10)
        
        # Get parameters from launch file
        self.num_runs = 5
        self.auto_increment = False
        self.base_distance = 30.0
        self.base_speed = 25.0
        self.speed_increment = 50.0

        # Data storage for results
        self.results = []

        self.get_logger().info('Extruder Calibrator node started.')
        self.get_logger().info(f"Configuration: {self.num_runs} runs, auto_increment={self.auto_increment}")

    def extrude(self, distance_mm, speed_sps):
        """
        Commands the extruder to run at a specific speed for a calculated duration.
        
        Args:
            distance_mm (float): The target distance of filament to extrude.
            speed_sps (float): The speed to run the stepper in steps per second.
        """
        if speed_sps <= 0:
            self.get_logger().error("Speed must be positive.")
            return False

        # 1. Calculate total steps needed
        total_steps = distance_mm * THEORETICAL_STEPS_PER_MM
        
        # 2. Calculate the required run time in seconds
        run_time_seconds = total_steps / speed_sps

        self.get_logger().info(f"Commanding extrusion of {distance_mm}mm.")
        self.get_logger().info(f"Calculated steps: {float(total_steps):.1f}")
        self.get_logger().info(f"Running motor at {speed_sps} steps/sec for {run_time_seconds:.2f} seconds.")

        # 3. Create and publish the speed message to start the motor
        start_msg = Float32()
        start_msg.data = float(speed_sps)
        self.publisher_.publish(start_msg)
        self.get_logger().info("Motor ON.")

        # 4. Wait for the calculated duration
        time.sleep(run_time_seconds)

        # 5. Publish a zero speed message to stop the motor
        stop_msg = Float32()
        stop_msg.data = 0.0
        self.publisher_.publish(stop_msg)
        self.get_logger().info("Motor OFF. Extrusion complete.")
        
        return True

    def get_user_input_float(self, prompt, min_value=0.0):
        """Get validated float input from user."""
        while True:
            try:
                value = float(input(prompt))
                if value < min_value:
                    print(f"Value must be >= {min_value}")
                    continue
                return value
            except ValueError:
                print("Please enter a valid number.")
            except KeyboardInterrupt:
                raise

    def run_calibration(self):
        """Run the full calibration sequence."""
        print("\n" + "="*50)
        print("EXTRUDER CALIBRATION SEQUENCE")
        print("="*50)
        print(f"Theoretical steps per mm: {THEORETICAL_STEPS_PER_MM:.2f}")
        print(f"Number of runs: {self.num_runs}")
        print(f"Auto increment mode: {self.auto_increment}")
        
        current_speed = self.base_speed
        
        for run_num in range(1, self.num_runs + 1):
            print(f"\n--- RUN {run_num}/{self.num_runs} ---")
            
            # Get parameters for this run
            if self.auto_increment:
                distance_mm = self.base_distance
                speed_sps = current_speed
                print(f"Auto mode: Distance = {distance_mm}mm, Speed = {speed_sps} steps/sec")
                current_speed += self.speed_increment
            else:
                print(f"Manual mode - please enter parameters for run {run_num}:")
                distance_mm = self.get_user_input_float("Enter distance to extrude (mm): ", 0.1)
                speed_sps = self.get_user_input_float("Enter speed (steps/sec): ", 1.0)
            
            # Perform the extrusion
            success = self.extrude(distance_mm, speed_sps)
            if not success:
                print("Extrusion failed, skipping this run.")
                continue
            
            # Get actual measurement from user
            print("\nPlease measure the actual filament extruded.")
            actual_distance = self.get_user_input_float("Enter actual distance extruded (mm): ", 0.0)
            
            # Calculate actual steps per mm
            if actual_distance > 0:
                actual_spm = (distance_mm / actual_distance) * THEORETICAL_STEPS_PER_MM
            else:
                actual_spm = float('inf')  # Handle division by zero case
                print("Warning: Actual distance is 0, cannot calculate steps per mm")
            
            # Store results
            self.results.append({
                'run': run_num,
                'requested_distance': distance_mm,
                'speed_sps': speed_sps,
                'actual_distance': actual_distance,
                'actual_spm': actual_spm
            })
            
            print(f"Calculated actual steps per mm: {actual_spm:.2f}")
            print(f"(Theoretical: {THEORETICAL_STEPS_PER_MM:.2f})")
        
        # Print summary
        self.print_summary()

    def print_summary(self):
        """Print a summary table of all results."""
        print("\n" + "="*80)
        print("CALIBRATION SUMMARY")
        print("="*80)
        print(f"{'Run':<4} {'Speed (sps)':<12} {'Req. Dist':<10} {'Act. Dist':<10} {'Act. SPM':<12} {'Error %':<10}")
        print("-" * 80)
        
        for result in self.results:
            if result['actual_spm'] != float('inf'):
                error_percent = ((result['actual_spm'] - THEORETICAL_STEPS_PER_MM) / THEORETICAL_STEPS_PER_MM) * 100
                error_str = f"{error_percent:+.1f}%"
            else:
                error_str = "N/A"
            
            print(f"{result['run']:<4} "
                  f"{result['speed_sps']:<12.0f} "
                  f"{result['requested_distance']:<10.1f} "
                  f"{result['actual_distance']:<10.1f} "
                  f"{result['actual_spm']:<12.2f} "
                  f"{error_str:<10}")
        
        # Calculate average if we have valid results
        valid_results = [r for r in self.results if r['actual_spm'] != float('inf')]
        if valid_results:
            avg_spm = sum(r['actual_spm'] for r in valid_results) / len(valid_results)
            print("-" * 80)
            print(f"Average actual steps per mm: {avg_spm:.2f}")
            print(f"Theoretical steps per mm: {THEORETICAL_STEPS_PER_MM:.2f}")
            print(f"Recommended calibration factor: {avg_spm:.2f}")


def main(args=None):
    rclpy.init(args=args)
    calibrator_node = ExtruderCalibrator()

    try:
        calibrator_node.run_calibration()
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user.")
    except Exception as e:
        calibrator_node.get_logger().error(f"Calibration failed: {str(e)}")
    finally:
        # Stop motor as a failsafe
        try:
            stop_msg = Float32()
            stop_msg.data = 0.0
            calibrator_node.publisher_.publish(stop_msg)
            calibrator_node.get_logger().info("Failsafe: Motor stopped.")
        except:
            pass
        
        calibrator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()