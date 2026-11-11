"""Measured bed height in robot Base coordinates, from a bed_probe result file."""

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.interpolate import RBFInterpolator
import yaml

# Transforming only segment endpoints leaves a chord error on a curved bed: about
# 0.095 mm for a 95 mm line on the first probed bed, 0.002 mm for 10 mm pieces.
MAX_SEGMENT_MM = 10.0


class BedSurface:
    """Thin-plate spline through the probed nozzle-touch heights, exact at every point."""

    def __init__(self, xy, z, extrapolation_mm, stamp='', source=''):
        """Fit Base XY -> bed Z; at least three non-collinear points."""
        xy = np.asarray(xy, dtype=float)
        z = np.asarray(z, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or z.shape != (len(xy),) or len(z) < 3:
            raise ValueError('Bed probe needs at least 3 points with X, Y and Z')
        if not (np.all(np.isfinite(xy)) and np.all(np.isfinite(z))):
            raise ValueError('Bed probe contains nonfinite values')
        if np.linalg.matrix_rank(xy - xy.mean(axis=0), tol=1.0) < 2:
            raise ValueError('Bed probe points are collinear')
        if not math.isfinite(extrapolation_mm) or extrapolation_mm < 0:
            raise ValueError('bed_probe_extrapolation_mm must be finite and nonnegative')
        self.model = RBFInterpolator(xy, z, kernel='thin_plate_spline')
        self.low = xy.min(axis=0) - extrapolation_mm
        self.high = xy.max(axis=0) + extrapolation_mm
        self.z = z
        self.stamp = stamp
        self.source = source
        self.orientation_note = ''

    @classmethod
    def load(cls, path, tcp_offset_mm, nozzle_rpy_deg, extrapolation_mm):
        """Reuse upright Base-frame measurements with the same calibrated nozzle TCP."""
        path = Path(path).expanduser().resolve()
        if not path.is_file():
            raise ValueError(f'bed_probe_file not found: {path} '
                             '(run bed_probe from the AxisForge root, or set it to "")')
        data = yaml.safe_load(path.read_text())
        orientation_note = ''
        for name, current in (('tcp_offset_mm', tcp_offset_mm),
                              ('nozzle_rpy_deg', nozzle_rpy_deg)):
            probed = np.asarray(data.get(name), dtype=float)
            if probed.shape != (3,) or not np.all(np.isfinite(probed)):
                raise ValueError(f'{path.name}: {name} must contain three finite numbers; '
                                 'probe the bed again')
            if np.allclose(probed, current, rtol=0, atol=1e-3):
                continue
            if name == 'nozzle_rpy_deg':
                roll, pitch, _ = np.deg2rad(probed)
                upright = math.isclose(math.cos(roll) * math.cos(pitch), -1.0,
                                       rel_tol=0, abs_tol=1e-10)
                calibrated_tip = not np.allclose(tcp_offset_mm, 0.0, rtol=0, atol=1e-3)
                # Upright touch points locate the bed in Base, independent of the later
                # print orientation. tool_pose() rotates the same TCP offset for printing.
                # A tilted touch can locate the nozzle rim instead of its center; without
                # a nozzle contact model it must not be reused at another orientation.
                if upright and calibrated_tip:
                    orientation_note = (f'upright probe RPY {probed.tolist()} reused at '
                                        f'print RPY {list(current)} with the same nozzle TCP')
                    continue
            if name == 'tcp_offset_mm':
                raise ValueError(f'{path.name} was probed with {name} {probed}, now '
                                 f'{list(current)}; probe the bed again')
            raise ValueError(f'{path.name} was probed with {name} {probed}, now '
                             f'{list(current)}; orientation changes require an upright '
                             'probe and a calibrated nonzero tcp_offset_mm; probe the bed again')
        points = data.get('points') or []
        xy = [point['nozzle_base_mm'][:2] for point in points]
        z = [point['bed_z_mm'] for point in points]
        surface = cls(xy, z, extrapolation_mm, str(data.get('stamp', '')), str(path))
        surface.orientation_note = orientation_note
        return surface

    def height(self, xy):
        """Bed Z under a robot Base XY; reject points beyond the probed area."""
        xy = np.asarray(xy, dtype=float)[:2]
        if np.any(xy < self.low) or np.any(xy > self.high):
            raise ValueError(f'Base XY ({xy[0]:.1f}, {xy[1]:.1f}) is outside the probed bed '
                             'area; probe a larger grid or move the print')
        return float(self.model(xy[None, :])[0])

    def describe(self):
        """Short provenance line for the startup log."""
        description = (f'{len(self.z)} points probed {self.stamp} ({self.source}); bed Z '
                       f'{self.z.min():.3f}..{self.z.max():.3f} mm, '
                       f'range {np.ptp(self.z):.3f} mm')
        return description + (f'; {self.orientation_note}' if self.orientation_note else '')


def subdivide(segments, max_length_mm=MAX_SEGMENT_MM):
    """Split long spatial segments into equal pieces with proportional E."""
    pieces = []
    for segment in segments:
        count = math.ceil(segment.length_mm / max_length_mm)
        if count <= 1:
            pieces.append(segment)
            continue
        nodes = np.linspace(segment.start, segment.end, count + 1)
        pitches = None
        if segment.start_pitch_deg is not None and segment.nozzle_pitch_deg is not None:
            pitches = np.linspace(segment.start_pitch_deg, segment.nozzle_pitch_deg, count + 1)
        pieces.extend(replace(segment, start=tuple(start), end=tuple(end),
                              delta_e=segment.delta_e / count,
                              start_pitch_deg=(segment.start_pitch_deg if pitches is None
                                               else float(pitches[index])),
                              nozzle_pitch_deg=(segment.nozzle_pitch_deg if pitches is None
                                                else float(pitches[index + 1])))
                      for index, (start, end) in enumerate(zip(nodes[:-1], nodes[1:])))
    return pieces
