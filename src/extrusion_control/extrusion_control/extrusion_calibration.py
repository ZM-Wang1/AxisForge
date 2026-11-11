"""Extruder drive constants: one theoretical steps/mm at every filament speed."""

import math


# Theoretical motor STEP pulses per filament mm (replaces the speed-dependent table).
STEPS_PER_MM = 1075.73
MAX_STEPPER_SPEED = 64000.0
# Firmware-counted moves use TIM1's 16-bit repetition counter.
MAX_MOVE_STEPS = 65536


def filament_speed_to_steps_s(filament_speed_mm_s):
    """Convert nonnegative filament feed to motor pulses, rejecting overflow."""
    if not math.isfinite(filament_speed_mm_s) or filament_speed_mm_s < 0:
        raise ValueError('Filament feed must be finite and nonnegative')
    speed = filament_speed_mm_s * STEPS_PER_MM
    if speed > MAX_STEPPER_SPEED:
        raise ValueError('Calculated extrusion exceeds 64000 steps/s; reduce the filament feed')
    return speed
