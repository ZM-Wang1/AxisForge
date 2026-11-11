"""Hardware-independent tests for the experimental Klipper capture adapter."""

from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest

from print_control.klipper_follower import (
    KlipperCaptureBuffer, MotionStreamError, main, validate_capture,
)


def batch(sequence=7, print_time=12.0, duration=0.25):
    """Build the companion exporter's protocol-v1 payload."""
    return {
        'protocol': 1, 'sequence': sequence, 'clock': 'klipper_print_time',
        'latest_motion_time': print_time + duration,
        'streams': {
            'toolhead': [{
                'print_time': print_time, 'duration': duration,
                'start_velocity': 20.0, 'acceleration': 0.0,
                'start_position': [10.0, 20.0, 30.0],
                'direction': [1.0, 0.0, 0.0],
            }],
        },
    }


class TestKlipperCaptureBuffer(unittest.TestCase):
    """Exercise timing preservation, bounded queues and fail-closed ingestion."""

    def test_explicit_gate(self):
        """Default construction must not enable the experimental path."""
        with self.assertRaisesRegex(MotionStreamError, 'disabled'):
            KlipperCaptureBuffer()

    def test_wrapped_accelerating_phase_preserves_absolute_time(self):
        """Use Klipper's polynomial, without unit conversion or retiming."""
        payload = batch()
        payload['streams']['toolhead'][0]['acceleration'] = 40.0
        buffer = KlipperCaptureBuffer(experimental=True)
        self.assertEqual(buffer.ingest({'params': payload}), 1)
        move = buffer.pop()
        self.assertEqual((move.print_time, move.end_time, move.sequence), (12, 12.25, 7))
        sample = move.sample(12.125)
        self.assertEqual(sample.print_time, 12.125)
        self.assertEqual(sample.position, (12.8125, 20, 30))
        self.assertEqual(sample.velocity, (25, 0, 0))
        self.assertEqual(sample.acceleration, (40, 0, 0))
        self.assertEqual(move.sample(12.25).position, (16.25, 20, 30))
        with self.assertRaisesRegex(MotionStreamError, 'outside'):
            move.sample(12.251)

    def test_deceleration_and_direction(self):
        """Preserve signed XYZ direction and deceleration to a stop."""
        payload = batch(duration=1)
        record = payload['streams']['toolhead'][0]
        record['acceleration'] = -20
        record['direction'] = [0, -1, 0]
        buffer = KlipperCaptureBuffer(experimental=True)
        buffer.ingest(payload)
        sample = buffer.pop().sample(13)
        self.assertEqual(sample.position, (10, 10, 30))
        self.assertEqual(sample.velocity, (0, 0, 0))

    def test_sequence_faults_survive_consumption(self):
        """Never hide a missing batch, replay or reconnect by emptying the FIFO."""
        for sequence in (7, 9, 0):
            with self.subTest(sequence=sequence):
                buffer = KlipperCaptureBuffer(experimental=True)
                buffer.ingest(batch())
                buffer.pop()
                with self.assertRaisesRegex(MotionStreamError, 'Sequence'):
                    buffer.ingest(batch(sequence, 12.25))
                with self.assertRaisesRegex(MotionStreamError, 'faulted'):
                    buffer.ingest(batch(8, 12.25))

    def test_overlap_with_consumed_phase_is_rejected(self):
        """Retain the timing watermark after phases leave the pending buffer."""
        buffer = KlipperCaptureBuffer(experimental=True)
        buffer.ingest(batch())
        buffer.pop()
        with self.assertRaisesRegex(MotionStreamError, 'overlap'):
            buffer.ingest(batch(8, 12.1))

    def test_gaps_are_not_filled(self):
        """A missing interval is not synthesized as movement or an idle hold."""
        buffer = KlipperCaptureBuffer(experimental=True)
        buffer.ingest(batch())
        buffer.ingest(batch(8, 13))
        first, second = buffer.pop(), buffer.pop()
        self.assertEqual((first.end_time, second.print_time), (12.25, 13))
        with self.assertRaises(MotionStreamError):
            first.sample(12.5)
        with self.assertRaises(MotionStreamError):
            second.sample(12.5)

    def test_record_and_horizon_limits(self):
        """Bound pending motion across messages, with no silent dropping."""
        for options, message, reason in (
                ({'max_records': 1}, batch(8, 12.25), 'record limit'),
                ({'max_buffer_s': 0.4}, batch(8, 12.25), 'horizon'),
                ({'max_buffer_s': 1}, batch(8, 14), 'horizon')):
            with self.subTest(options=options):
                buffer = KlipperCaptureBuffer(experimental=True, **options)
                buffer.ingest(batch())
                with self.assertRaisesRegex(MotionStreamError, reason):
                    buffer.ingest(message)
                self.assertEqual(len(buffer), 1)
                with self.assertRaisesRegex(MotionStreamError, 'faulted'):
                    buffer.pop()

    def test_extruder_queues_are_not_cartesian_motion(self):
        """Ignore native extrusion coordinates, but include them in batch bounds."""
        payload = batch()
        payload['streams'] = {'extruder': [{'native_coordinate': 10}]}
        buffer = KlipperCaptureBuffer(experimental=True, max_records=1)
        self.assertEqual(buffer.ingest(payload), 0)
        buffer.ingest(batch(8))
        self.assertEqual(len(buffer), 1)
        payload['streams']['extruder'] *= 2
        with self.assertRaisesRegex(MotionStreamError, 'including ignored'):
            KlipperCaptureBuffer(experimental=True, max_records=1).ingest(payload)

    def test_invalid_envelopes(self):
        """Reject alternate clocks, malformed envelopes and invalid sequences."""
        for key, value in (
                ('protocol', 2), ('protocol', True), ('clock', 'ros_time'),
                ('sequence', True), ('sequence', -1), ('sequence', 7.0),
                ('streams', []), ('streams', {'toolhead': {}}),
                ('streams', {}), ('streams', {'toolhead': []}),
                ('latest_motion_time', None), ('latest_motion_time', float('nan'))):
            with self.subTest(key=key, value=value):
                payload = batch()
                payload[key] = value
                with self.assertRaises(MotionStreamError):
                    KlipperCaptureBuffer(experimental=True).ingest(payload)
        for message in ([], {'params': []}, {'params': {'header': []}}):
            with self.assertRaises(MotionStreamError):
                KlipperCaptureBuffer(experimental=True).ingest(message)

    def test_invalid_phases(self):
        """Reject nonfinite, malformed, backwards and overflowing phase data."""
        for key, value in (
                ('duration', 0), ('duration', -1), ('duration', '0.25'),
                ('print_time', -1), ('print_time', 1e30),
                ('start_velocity', True), ('start_velocity', -1),
                ('acceleration', float('inf')), ('acceleration', -1000),
                ('start_position', [1, 2]), ('direction', [1, float('nan'), 0]),
                ('direction', [1e308, 0, 0])):
            with self.subTest(key=key, value=value):
                payload = batch()
                payload['streams']['toolhead'][0][key] = value
                with self.assertRaises(MotionStreamError):
                    KlipperCaptureBuffer(experimental=True).ingest(payload)

    def test_invalid_batch_is_atomic(self):
        """A valid prefix must not become consumable when its batch is invalid."""
        payload = batch()
        payload['streams']['toolhead'].append(
            deepcopy(payload['streams']['toolhead'][0]))
        buffer = KlipperCaptureBuffer(experimental=True)
        with self.assertRaisesRegex(MotionStreamError, 'overlap'):
            buffer.ingest(payload)
        self.assertEqual(len(buffer), 0)
        with self.assertRaisesRegex(MotionStreamError, 'faulted'):
            buffer.pop()

    def test_underflow_is_an_explicit_fault(self):
        """Exhaustion cannot be mistaken for a safe live hold."""
        buffer = KlipperCaptureBuffer(experimental=True)
        with self.assertRaisesRegex(MotionStreamError, 'underrun'):
            buffer.pop()
        with self.assertRaisesRegex(MotionStreamError, 'faulted'):
            buffer.ingest(batch())


class TestCaptureCommand(unittest.TestCase):
    """Run the offline entry point without ROS, IK models or robot connections."""

    def setUp(self):
        """Create an isolated capture file for each test."""
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / 'capture.ndjson'

    def write_capture(self, *messages):
        """Write one complete batch per line, as required by the capture format."""
        self.path.write_text(
            ''.join(json.dumps(message) + '\n' for message in messages), encoding='utf-8')

    def test_cli_validates_only(self):
        """Report gaps and source times without claiming execution readiness."""
        self.write_capture({'params': batch()}, batch(8, 13))
        output = io.StringIO()
        with redirect_stdout(output):
            main(['--experimental-klipper-follower', '--input', str(self.path)])
        summary = json.loads(output.getvalue())
        self.assertEqual(summary, {
            'mode': 'capture_validation_only', 'batches': 2, 'toolhead_phases': 2,
            'gaps': 1, 'first_print_time': 12.0, 'last_print_time': 13.25,
            'live_execution_available': False,
        })

    def test_cli_requires_opt_in_and_emits_no_success_on_failure(self):
        """The default CLI and malformed captures must exit nonzero."""
        self.write_capture(batch())
        for flags, content in (
                ([], self.path.read_text(encoding='utf-8')),
                (['--experimental-klipper-follower'], '{"params":\n')):
            with self.subTest(flags=flags):
                self.path.write_text(content, encoding='utf-8')
                output, errors = io.StringIO(), io.StringIO()
                with redirect_stdout(output), redirect_stderr(errors):
                    with self.assertRaises(SystemExit) as raised:
                        main([*flags, '--input', str(self.path)])
                self.assertEqual(raised.exception.code, 1)
                self.assertEqual(output.getvalue(), '')
                self.assertIn('rejected', errors.getvalue())

    def test_line_bound_and_empty_toolhead(self):
        """Reject oversized, deeply nested and non-motion captures explicitly."""
        self.write_capture(batch())
        with self.assertRaisesRegex(MotionStreamError, 'byte limit'):
            validate_capture(self.path, experimental=True, max_line_bytes=32)
        for content in ('', '{"params": []}\n', '[]\n',
                        '[' * 2000 + '0' + ']' * 2000 + '\n'):
            self.path.write_text(content, encoding='utf-8')
            with self.assertRaises(MotionStreamError):
                validate_capture(self.path, experimental=True)
        payload = batch()
        payload['streams'] = {'extruder': [{'native_coordinate': 10}]}
        self.write_capture(payload)
        with self.assertRaisesRegex(MotionStreamError, 'no toolhead'):
            validate_capture(self.path, experimental=True)


if __name__ == '__main__':
    unittest.main()
