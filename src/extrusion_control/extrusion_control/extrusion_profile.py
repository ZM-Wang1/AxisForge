"""Feed-forward filament motion planned together with the robot trajectory (Klipper style)."""

from dataclasses import dataclass

import numpy as np

from extrusion_control.extrusion_calibration import MAX_STEPPER_SPEED, STEPS_PER_MM


@dataclass(frozen=True)
class ExtrusionProfile:
    """Signed cumulative STEP pulses against time from the trajectory start."""

    times: np.ndarray
    steps: np.ndarray

    @classmethod
    def plan(cls, times, position_mm, speed_mm_s, advance_s, direction):
        """Apply pressure advance E + K * dE/dt, then convert filament to signed steps.

        A rest-to-rest motion ends at zero speed, so the advance cancels there and the
        total equals the planned filament: the extruder briefly reverses while decelerating.
        """
        steps = direction * (position_mm + advance_s * speed_mm_s) * STEPS_PER_MM
        peak = float(np.max(np.abs(np.diff(steps)) / np.diff(times)))
        if peak > MAX_STEPPER_SPEED:
            raise ValueError(f'Planned extrusion needs {peak:.0f} steps/s; '
                             f'firmware limit is {MAX_STEPPER_SPEED:.0f}')
        return cls(np.asarray(times, dtype=float), steps)

    @property
    def duration_s(self):
        """Return the time of the last sample."""
        return float(self.times[-1])

    def steps_at(self, elapsed_s):
        """Return the planned position; constant before the start and after the end."""
        return float(np.interp(elapsed_s, self.times, self.steps))
