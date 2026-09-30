"""Shared filament-speed calibration, originally measured for G-code printing."""

import math

import numpy as np


# Input: filament feed in mm/s. Output: motor STEP pulses per filament mm.
CALIBRATION_SPEEDS = (0.91, 1.129, 2.258, 3.387, 4.516, 5.645, 6.774, 7.9, 9.032, 10.161)
CALIBRATION_STEPS_PER_MM = (
    196.83, 201.30, 201.30, 212.91, 221.43, 249.52, 250.68, 288.82, 288.82, 301.95,
)
MAX_STEPPER_SPEED = 64000.0


def steps_per_mm(filament_speed_mm_s):
    """Interpolate the original table, retaining its endpoint and <=0 behavior."""
    if not math.isfinite(filament_speed_mm_s):
        raise ValueError('Filament speed must be finite')
    if filament_speed_mm_s <= 0:
        return 221.43
    return float(np.interp(
        filament_speed_mm_s, CALIBRATION_SPEEDS, CALIBRATION_STEPS_PER_MM,
    ))


def filament_speed_to_steps_s(filament_speed_mm_s):
    """Convert nonnegative filament feed to motor pulses, rejecting overflow."""
    if not math.isfinite(filament_speed_mm_s) or filament_speed_mm_s < 0:
        raise ValueError('Filament feed must be finite and nonnegative')
    speed = filament_speed_mm_s * steps_per_mm(filament_speed_mm_s)
    if speed > MAX_STEPPER_SPEED:
        raise ValueError('Calculated extrusion exceeds 64000 steps/s; reduce filament_per_mm')
    return speed


def outside_calibration_range(filament_speed_mm_s):
    """Identify positive feed rates for which the table holds an endpoint."""
    return (filament_speed_mm_s > 0 and
            not CALIBRATION_SPEEDS[0] <= filament_speed_mm_s <= CALIBRATION_SPEEDS[-1])
