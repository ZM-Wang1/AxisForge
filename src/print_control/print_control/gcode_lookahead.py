"""Continuous path geometry, material coordinates and acceleration look-ahead."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class ContinuousPath:
    """One same-sign run; material is positive cumulative filament distance."""

    segments: tuple
    points: np.ndarray
    tangents: np.ndarray
    source_indices: np.ndarray
    distances: np.ndarray
    material: np.ndarray
    speeds: np.ndarray
    times: np.ndarray
    speed_limits: np.ndarray

    @property
    def length_mm(self):
        """Return sampled arc length."""
        return float(self.distances[-1])

    @property
    def direction(self):
        """Return the common extrusion sign, or zero for travel."""
        return float(np.sign(self.segments[0].delta_e))

    def closest_source_point(self, point, interval):
        """Find the nearest original segment near this sampled path interval."""
        first = max(0, int(self.source_indices[interval]) - 1)
        last = min(len(self.segments), int(self.source_indices[interval + 1]) + 2)
        candidates = []
        for segment in self.segments[first:last]:
            start, end = np.asarray(segment.start), np.asarray(segment.end)
            delta = end - start
            fraction = np.clip((point - start) @ delta / (delta @ delta), 0, 1)
            candidates.append(start + fraction * delta)
        return min(candidates, key=lambda candidate: np.linalg.norm(point - candidate))

    def filament(self, times):
        """Planned filament position (mm) and speed (mm/s) at trajectory times.

        plan_speeds accelerates uniformly within each sample interval and material is
        linear in arc length there, so this follows the planned timing exactly.
        """
        times = np.clip(times, self.times[0], self.times[-1])
        index = np.clip(np.searchsorted(self.times, times, side='right') - 1,
                        0, len(self.times) - 2)
        elapsed = times - self.times[index]
        start_speed = self.speeds[index]
        acceleration = (self.speeds[index + 1] - start_speed) / np.diff(self.times)[index]
        distance = start_speed * elapsed + acceleration * elapsed**2 / 2
        ratio = np.diff(self.material)[index] / np.diff(self.distances)[index]
        return (self.material[index] + ratio * distance,
                ratio * (start_speed + acceleration * elapsed))


def plan_speeds(distances, limits, acceleration):
    """Backward braking and forward acceleration passes over a complete run."""
    speeds = np.asarray(limits, dtype=float).copy()
    speeds[[0, -1]] = 0.0
    for index in range(len(speeds) - 2, -1, -1):
        reach = math.sqrt(speeds[index + 1]**2 + 2 * acceleration *
                          (distances[index + 1] - distances[index]))
        speeds[index] = min(speeds[index], reach)
    for index in range(1, len(speeds)):
        reach = math.sqrt(speeds[index - 1]**2 + 2 * acceleration *
                          (distances[index] - distances[index - 1]))
        speeds[index] = min(speeds[index], reach)
    sums = speeds[:-1] + speeds[1:]
    if np.any(sums <= 0):
        raise ValueError('Two consecutive path samples have zero allowed speed')
    times = np.r_[0.0, np.cumsum(2 * np.diff(distances) / sums)]
    return speeds, times


def build_continuous_path(segments, speed_limits, spacing, blend_tolerance, corner_speed,
                          acceleration, max_feed, max_points):
    """Round corners within tolerance and retain source E through arc-length changes."""
    segments = tuple(segments)
    starts = np.array([s.start for s in segments])
    ends = np.array([s.end for s in segments])
    lengths = np.linalg.norm(ends - starts, axis=1)
    if np.any(lengths <= 1e-9) or not np.allclose(ends[:-1], starts[1:], atol=1e-7, rtol=0):
        raise ValueError('Continuous path requires connected nonzero spatial segments')
    signs = np.sign([s.delta_e for s in segments])
    if np.any(signs != signs[0]):
        raise ValueError('A continuous run must have a single extrusion direction')
    vectors = (ends - starts) / lengths[:, None]
    source_s = np.r_[0.0, np.cumsum(lengths)]
    source_e = np.r_[0.0, np.cumsum([abs(s.delta_e) for s in segments])]
    trim = np.zeros(len(segments) + 1)
    stop = np.zeros(len(segments) + 1, dtype=bool)
    for index in range(1, len(segments)):
        cosine = float(np.clip(vectors[index - 1] @ vectors[index], -1, 1))
        # Pieces of one source line (bed subdivision) bend only by the bed curvature
        # (~0.1 deg); a corner blend there would cap every piece at max_corner_speed.
        same_line = segments[index - 1].source_line == segments[index].source_line
        if cosine > 1 - 1e-10 or same_line:
            continue
        # Near reversals cannot be passed at a finite nonzero speed in a tiny blend.
        if cosine < -0.95 or blend_tolerance == 0:
            stop[index] = True
            continue
        sin_half = math.sqrt((1 - cosine) / 2)
        trim[index] = min(2 * blend_tolerance / sin_half,
                          lengths[index - 1] / 4, lengths[index] / 4)

    points, tangents, stations, caps, sources = [], [], [], [], []

    def append(point, tangent, station, cap):
        source = min(int(np.searchsorted(source_s, station, side='right')) - 1, len(segments) - 1)
        source = max(0, source)
        cap = min(cap, speed_limits[source])
        if points and np.linalg.norm(point - points[-1]) < 1e-10:
            caps[-1] = min(caps[-1], cap)
            return
        if len(points) >= max_points:
            raise ValueError(f'Continuous path exceeds max_continuous_points={max_points}')
        points.append(np.asarray(point))
        tangents.append(np.asarray(tangent))
        stations.append(station)
        caps.append(cap)
        sources.append(source)

    for index, segment in enumerate(segments):
        start = starts[index] + trim[index] * vectors[index]
        end = ends[index] - trim[index + 1] * vectors[index]
        s0, s1 = source_s[index] + trim[index], source_s[index + 1] - trim[index + 1]
        intervals = max(2, math.ceil((s1 - s0) / spacing))
        for fraction in np.linspace(0, 1, intervals + 1):
            cap = speed_limits[index]
            if (fraction == 0 and stop[index]) or (fraction == 1 and stop[index + 1]):
                cap = 0.0
            append(start + fraction * (end - start), vectors[index],
                   s0 + fraction * (s1 - s0), cap)
        distance = trim[index + 1]
        if distance == 0:
            continue
        corner = ends[index]
        after = corner + distance * vectors[index + 1]
        second = 2 * (after - 2 * corner + end)
        intervals = max(4, math.ceil(2 * distance / spacing))
        for u in np.linspace(0, 1, intervals + 1):
            derivative = 2 * ((1 - u) * (corner - end) + u * (after - corner))
            norm = np.linalg.norm(derivative)
            curvature = np.linalg.norm(np.cross(derivative, second)) / norm**3
            cap = min(speed_limits[index], speed_limits[index + 1], corner_speed)
            if curvature > 0:
                cap = min(cap, math.sqrt(acceleration / math.sqrt(2) / curvature))
            append((1 - u)**2 * end + 2 * u * (1 - u) * corner + u**2 * after,
                   derivative / norm, source_s[index + 1] - distance + 2 * distance * u, cap)

    points = np.asarray(points)
    distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    material = np.interp(stations, source_s, source_e)
    caps = np.asarray(caps)
    ratios = np.diff(material) / np.diff(distances)
    for index, ratio in enumerate(ratios):
        if ratio > 0:
            caps[index:index + 2] = np.minimum(caps[index:index + 2], max_feed / ratio)
    # Reserve acceleration for simultaneous curvature and tangential acceleration.
    speeds, times = plan_speeds(distances, caps, acceleration / math.sqrt(2))
    return ContinuousPath(segments, points, np.asarray(tangents), np.asarray(sources),
                          distances, material, speeds, times, np.asarray(caps))
