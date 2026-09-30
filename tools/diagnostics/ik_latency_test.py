import time
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import gc

def test_ik_method(robot_dh, poses, joint_positions, method_name, ik_function):
    """Test a specific IK method and return latency statistics"""
    print(f"\nTesting {method_name}...")

    # Clear memory before test
    gc.collect()

    latencies = []
    iterations = len(poses) - 1

    for i in range(1, iterations + 1):
        current_q = joint_positions[i-1]
        target_pose = poses[i]

        start_time = time.perf_counter()
        ik_function(target_pose, q0=current_q)
        end_time = time.perf_counter()

        latencies.append(end_time - start_time)

    # Convert to milliseconds
    latencies_ms = np.array(latencies) * 1000

    avg_latency = np.mean(latencies_ms)
    min_latency = np.min(latencies_ms)
    max_latency = np.max(latencies_ms)
    std_latency = np.std(latencies_ms)
    
    print(f"{method_name} Results:")
    print(f"  Average latency: {avg_latency:.4f} ms")
    print(f"  Min latency: {min_latency:.4f} ms")
    print(f"  Max latency: {max_latency:.4f} ms")
    print(f"  Std deviation: {std_latency:.4f} ms")

    # Clear latencies array and force garbage collection
    del latencies
    del latencies_ms
    gc.collect()

    return {
        'method': method_name,
        'avg': avg_latency,
        'min': min_latency,
        'max': max_latency,
        'std': std_latency
    }

def main():
    print("Initializing robot models...")
    robot = rtb.models.UR5()
    robot_dh = rtb.models.DH.UR5()

    # Generate test poses (same for all methods)
    print("Generating test poses...")
    start_pose = [-0.2, -0.2, 0.200, 0, np.pi, 0]
    start_se3 = sm.SE3(start_pose[:3]) * sm.SE3.RPY(start_pose[3:])
    start_q, _, _, _, _  = robot_dh.ik_LM(start_se3)

    poses = []
    joint_positions = []

    iterations = 1000
    np.random.seed(42)  # Ensure reproducible results

    for i in range(iterations + 1):
        x_offset = np.random.uniform(-0.05, 0.05)
        y_offset = np.random.uniform(-0.05, 0.05)
        z_offset = np.random.uniform(-0.05, 0.05)

        pose = sm.SE3([start_pose[0] + x_offset, start_pose[1] + y_offset, start_pose[2] + z_offset]) * sm.SE3.RPY(start_pose[3:])
        target_q, _, _, _, _ = robot_dh.ik_LM(pose, q0=start_q)

        poses.append(pose)
        joint_positions.append(target_q)

    print(f"Generated {len(poses)} poses for testing")

    results = []
    results.append(test_ik_method(
        robot_dh, poses, joint_positions, 
        "ik_LM", 
        robot_dh.ik_LM
    ))

    time.sleep(0.1)
    gc.collect()

    results.append(test_ik_method(
        robot_dh, poses, joint_positions, 
        "ik_GN", 
        robot_dh.ik_GN
    ))

    time.sleep(0.1)
    gc.collect()

    results.append(test_ik_method(
        robot_dh, poses, joint_positions, 
        "ik_NR", 
        robot_dh.ik_NR
    ))

    time.sleep(0.1)
    gc.collect()

    results.append(test_ik_method(
        robot_dh, poses, joint_positions, 
        "ikine_LM", 
        robot_dh.ikine_LM
    ))

    time.sleep(0.1)
    gc.collect()

    results.append(test_ik_method(
        robot_dh, poses, joint_positions, 
        "ikine_GN", 
        robot_dh.ikine_GN
    ))

    time.sleep(0.1)
    gc.collect()

    results.append(test_ik_method(
        robot_dh, poses, joint_positions, 
        "ikine_NR", 
        robot_dh.ikine_NR
    ))

    print("\n" + "="*60)
    print("SUMMARY COMPARISON")
    print("="*60)
    print(f"{'Method':<20} {'Avg (ms)':<12} {'Min (ms)':<12} {'Max (ms)':<12} {'Std (ms)':<12}")
    print("-" * 68)
    
    for result in results:
        print(f"{result['method']:<20} {result['avg']:<12.4f} {result['min']:<12.4f} {result['max']:<12.4f} {result['std']:<12.4f}")

    fastest = min(results, key=lambda x: x['avg'])
    print(f"\nFastest method: {fastest['method']} with {fastest['avg']:.4f} ms average latency")

    print("\nRelative Performance (compared to fastest):")
    for result in results:
        ratio = result['avg'] / fastest['avg']
        print(f"{result['method']:<20}: {ratio:.2f}x")

if __name__ == "__main__":
    main()
