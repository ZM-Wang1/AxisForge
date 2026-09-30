"""Estimate filament feed from measured forward progress along one print line."""

from collections import deque
import math

from extrusion_control.extrusion_calibration import filament_speed_to_steps_s


class ProgressExtrusion:
    """Convert timestamped TCP progress to calibrated, nonnegative extrusion."""

    def __init__(self, filament_per_mm, length_mm, smoothing_s):
        """Set material per path length and a short velocity averaging window."""
        if (not all(math.isfinite(x) for x in (filament_per_mm, length_mm, smoothing_s))
                or filament_per_mm < 0 or length_mm <= 0 or smoothing_s <= 0):
            raise ValueError('Invalid extrusion ratio, path length or smoothing interval')
        self.filament_per_mm = filament_per_mm
        self.length_mm = length_mm
        self.smoothing_s = smoothing_s
        self.samples = deque()
        self.progress_mm = 0.0
        self.tcp_speed_mm_s = 0.0
        self.filament_speed_mm_s = 0.0
        self.steps_s = 0.0

    def update(self, progress_mm, timestamp_s):
        """Use forward-only progress; stop on a stationary or reversing sample."""
        if not math.isfinite(progress_mm) or not math.isfinite(timestamp_s):
            raise ValueError('Extrusion feedback must contain finite progress and time')
        if self.samples and timestamp_s <= self.samples[-1][0]:
            raise ValueError('Extrusion feedback timestamps must increase')
        progress = max(self.progress_mm, min(max(progress_mm, 0.0), self.length_mm))
        delta = progress - self.progress_mm
        self.progress_mm = progress
        self.samples.append((timestamp_s, progress))
        # Keep the sample immediately before the window boundary for stable timing.
        while len(self.samples) > 2 and self.samples[1][0] <= timestamp_s - self.smoothing_s:
            self.samples.popleft()
        self.tcp_speed_mm_s = 0.0
        if len(self.samples) > 1 and delta > 0 and progress < self.length_mm:
            first_time, first_progress = self.samples[0]
            self.tcp_speed_mm_s = (progress - first_progress) / (timestamp_s - first_time)
        else:
            # Do not carry a nonzero average across a stop, or repay missing feed in place.
            self.samples.clear()
            self.samples.append((timestamp_s, progress))
        self.filament_speed_mm_s = self.tcp_speed_mm_s * self.filament_per_mm
        self.steps_s = filament_speed_to_steps_s(self.filament_speed_mm_s)
        return self.steps_s
