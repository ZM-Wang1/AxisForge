import numpy as np

def steps_per_mm(extrusion_speed_mmps):
    """Return calibrated steps/mm based on extrusion speed."""
    if extrusion_speed_mmps <= 0:
        return 221.43
    
    # Calibration data: (estimated_speed_mmps, actual_steps_per_mm)
    speeds = np.array([0.91, 1.129, 2.258, 3.387, 4.516, 5.645, 6.774, 7.9, 9.032, 10.161])
    spms = np.array([196.83, 201.30, 201.30, 212.91, 221.43, 249.52, 250.68, 288.82, 288.82, 301.95])

    return float(np.interp(extrusion_speed_mmps, speeds, spms))

def main():
    test_speeds = list(range(1, 50, 1))
    for speed in test_speeds:
        steps_per_mm_value = steps_per_mm(speed)
        stepper_speed = speed * steps_per_mm_value
        print(f"Extrusion Speed: {speed} mm/s, Steps per mm: {steps_per_mm_value:.2f}, Stepper Speed: {stepper_speed:.2f} steps/s")

if __name__ == "__main__":
    main()