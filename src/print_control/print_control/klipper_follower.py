"""Experimental, hardware-free adapter for Klipper diagnostic trapq captures."""

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys


class MotionStreamError(ValueError):
    """Reject an invalid or discontinuous capture without attempting recovery."""


def number(value, name):
    """Require finite JSON numbers, not booleans or numeric strings."""
    if type(value) not in (int, float):
        raise MotionStreamError(f'{name} must be a finite number')
    try:
        result = float(value)
    except OverflowError as error:
        raise MotionStreamError(f'{name} is too large') from error
    if not math.isfinite(result):
        raise MotionStreamError(f'{name} must be finite')
    return result


def vector(value, name):
    """Read exactly three numeric components in the exporter coordinate frame."""
    if not isinstance(value, list) or len(value) != 3:
        raise MotionStreamError(f'{name} must be a three-number array')
    return tuple(number(component, name) for component in value)


@dataclass(frozen=True)
class CartesianSample:
    """Toolhead state in mm and seconds, with an absolute Klipper print time."""

    print_time: float
    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class TimedToolheadMove:
    """A Klipper constant-acceleration phase; never independently retimed."""

    sequence: int
    print_time: float
    duration: float
    start_velocity: float
    acceleration: float
    start_position: tuple
    direction: tuple

    @property
    def end_time(self):
        """Return the absolute end in the same print-time clock."""
        return self.print_time + self.duration

    def sample(self, print_time):
        """Evaluate this phase only; do not extrapolate or fill queue gaps."""
        print_time = number(print_time, 'sample print_time')
        if not self.print_time <= print_time <= self.end_time:
            raise MotionStreamError('Sample lies outside the motion phase')
        elapsed = print_time - self.print_time
        distance = elapsed * (self.start_velocity + 0.5 * self.acceleration * elapsed)
        speed = self.start_velocity + self.acceleration * elapsed
        return CartesianSample(
            print_time,
            tuple(p + d * distance for p, d in zip(self.start_position, self.direction)),
            tuple(d * speed for d in self.direction),
            tuple(d * self.acceleration for d in self.direction),
        )

    @classmethod
    def from_record(cls, record, sequence):
        """Validate one toolhead record, excluding extruder-native coordinates."""
        if not isinstance(record, dict):
            raise MotionStreamError('toolhead record must be an object')
        required = ('print_time', 'duration', 'start_velocity', 'acceleration',
                    'start_position', 'direction')
        if any(key not in record for key in required):
            raise MotionStreamError('toolhead record is missing required fields')
        move = cls(
            sequence, *(number(record[key], key) for key in required[:4]),
            vector(record['start_position'], 'start_position'),
            vector(record['direction'], 'direction'),
        )
        if move.print_time < 0 or move.duration <= 0:
            raise MotionStreamError('print_time must be nonnegative and duration positive')
        if not math.isfinite(move.end_time) or move.end_time <= move.print_time:
            raise MotionStreamError('Motion end time is not finite or representably increasing')
        end_velocity = move.start_velocity + move.acceleration * move.duration
        if move.start_velocity < 0 or end_velocity < -1e-9:
            raise MotionStreamError('Path speed must not become negative within a phase')
        for stamp in (move.print_time, move.end_time):
            sample = move.sample(stamp)
            if not all(math.isfinite(v) for v in
                       (*sample.position, *sample.velocity, *sample.acceleration)):
                raise MotionStreamError('Motion evaluation overflowed')
        return move


class KlipperCaptureBuffer:
    """Bounded FIFO with transactional ingestion and fail-closed sequence errors."""

    def __init__(self, *, experimental=False, max_records=4096, max_buffer_s=10.0):
        """Require deliberate enablement; no ROS, robot or extrusion dependencies."""
        if experimental is not True:
            raise MotionStreamError('Experimental Klipper follower adapter is disabled')
        if type(max_records) is not int or max_records <= 0:
            raise MotionStreamError('max_records must be a positive integer')
        self.max_buffer_s = number(max_buffer_s, 'max_buffer_s')
        if self.max_buffer_s <= 0:
            raise MotionStreamError('max_buffer_s must be positive')
        self.max_records = max_records
        self._moves = deque()
        self._last_sequence = None
        self._last_end = None
        self._failure = None

    def _check_failed(self):
        """Do not resume after losing the stream's ordering or timing contract."""
        if self._failure is not None:
            raise MotionStreamError(f'Adapter faulted: {self._failure}; start a new capture')

    def ingest(self, message):
        """Accept a decoded batch payload or its websocket ``params`` wrapper."""
        self._check_failed()
        try:
            return self._ingest(message)
        except MotionStreamError as error:
            self._failure = str(error)
            raise

    def _ingest(self, message):
        if not isinstance(message, dict):
            raise MotionStreamError('Batch must be an object')
        payload = message.get('params', message)
        if not isinstance(payload, dict):
            raise MotionStreamError('Batch params must be an object')
        if type(payload.get('protocol')) is not int or payload['protocol'] != 1:
            raise MotionStreamError('Only experimental protocol 1 is supported')
        if payload.get('clock') != 'klipper_print_time':
            raise MotionStreamError('Expected klipper_print_time clock')
        sequence = payload.get('sequence')
        if type(sequence) is not int or sequence < 0:
            raise MotionStreamError('sequence must be a nonnegative integer')
        if self._last_sequence is not None and sequence != self._last_sequence + 1:
            raise MotionStreamError('Sequence gap, duplicate or reset; start a new capture')
        if number(payload.get('latest_motion_time'), 'latest_motion_time') < 0:
            raise MotionStreamError('latest_motion_time must be nonnegative')
        streams = payload.get('streams')
        if not isinstance(streams, dict) or not streams or not all(
                isinstance(name, str) and isinstance(records, list) and records
                for name, records in streams.items()):
            raise MotionStreamError('streams must map queue names to nonempty record arrays')
        if sum(len(records) for records in streams.values()) > self.max_records:
            raise MotionStreamError('Batch record limit exceeded (including ignored queues)')
        # Extruder queues are intentionally not interpreted as Cartesian motion.
        moves = [TimedToolheadMove.from_record(record, sequence)
                 for record in streams.get('toolhead', [])]
        if len(self._moves) + len(moves) > self.max_records:
            raise MotionStreamError('Toolhead buffer record limit exceeded')
        previous_end = self._last_end
        for move in moves:
            if previous_end is not None and move.print_time < previous_end - 1e-9:
                raise MotionStreamError('Toolhead phases overlap or move backwards in time')
            previous_end = move.end_time
        first = self._moves[0] if self._moves else (moves[0] if moves else None)
        if first is not None and previous_end - first.print_time > self.max_buffer_s:
            raise MotionStreamError('Toolhead buffer time horizon exceeded')
        self._moves.extend(moves)
        self._last_sequence = sequence
        self._last_end = previous_end
        return len(moves)

    def pop(self):
        """Consume one intact timed phase; underflow is an explicit latched fault."""
        self._check_failed()
        if not self._moves:
            self._failure = 'Toolhead buffer underrun'
            raise MotionStreamError(self._failure)
        return self._moves.popleft()

    def __len__(self):
        """Return the number of unconsumed toolhead phases."""
        return len(self._moves)


def validate_capture(path, *, max_line_bytes=1048576, **buffer_options):
    """Stream bounded NDJSON records; report validation, not robot readiness."""
    if type(max_line_bytes) is not int or max_line_bytes <= 0:
        raise MotionStreamError('max_line_bytes must be a positive integer')
    buffer = KlipperCaptureBuffer(**buffer_options)
    batches = phases = gaps = 0
    first_time = last_end = None
    with Path(path).open('rb') as source:
        while True:
            line = source.readline(max_line_bytes + 1)
            if not line:
                break
            batches += 1
            if len(line) > max_line_bytes:
                raise MotionStreamError(f'Line {batches}: byte limit exceeded')
            try:
                message = json.loads(line)
                buffer.ingest(message)
                while buffer:
                    move = buffer.pop()
                    if first_time is None:
                        first_time = move.print_time
                    if last_end is not None and move.print_time > last_end + 1e-9:
                        gaps += 1
                    last_end = move.end_time
                    phases += 1
            except (ValueError, UnicodeError, RecursionError) as error:
                raise MotionStreamError(f'Line {batches}: {error}') from error
    if not phases:
        raise MotionStreamError('Capture contains no toolhead motion')
    return {
        'mode': 'capture_validation_only',
        'batches': batches, 'toolhead_phases': phases, 'gaps': gaps,
        'first_print_time': first_time, 'last_print_time': last_end,
        'live_execution_available': False,
    }


def main(args=None):
    """Validate an explicit offline capture; never connect to or command hardware."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experimental-klipper-follower', action='store_true',
                        help='enable capture validation only, not live execution')
    parser.add_argument('--input', required=True, help='NDJSON batches, one per line')
    parser.add_argument('--max-records', type=int, default=4096)
    parser.add_argument('--max-buffer-s', type=float, default=10.0)
    parser.add_argument('--max-line-bytes', type=int, default=1048576)
    options = parser.parse_args(args)
    try:
        summary = validate_capture(
            options.input, experimental=options.experimental_klipper_follower,
            max_records=options.max_records, max_buffer_s=options.max_buffer_s,
            max_line_bytes=options.max_line_bytes,
        )
    except (OSError, MotionStreamError) as error:
        print(f'Klipper capture rejected: {error}', file=sys.stderr)
        raise SystemExit(1) from error
    print(json.dumps(summary, allow_nan=False))


if __name__ == '__main__':
    main()
