"""Parse linear printer G-code into immutable, ROS-independent toolpath segments."""

from dataclasses import dataclass
import math
import re


WORD = re.compile(r'([A-Za-z])\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))?')


@dataclass(frozen=True)
class ToolpathSegment:
    """Machine coordinates in mm, filament delta in mm, feed in mm/s."""

    start: tuple
    end: tuple
    delta_e: float
    feed_mm_s: float
    kind: str
    source_line: int
    layer: int | None = None
    dwell_s: float = 0.0

    def __post_init__(self):
        """Reject overflow in resolved coordinates, feed or material increments."""
        values = (*self.start, *self.end, self.delta_e, self.feed_mm_s, self.dwell_s)
        if not all(math.isfinite(value) for value in values) or not math.isfinite(self.length_mm):
            raise ValueError('Resolved segment contains nonfinite values')

    @property
    def length_mm(self):
        """Return the geometric length, independent of extrusion."""
        return math.dist(self.start, self.end)


class GCodeParser:
    """Maintain Marlin-style modal state without publishing or moving anything."""

    def __init__(self, ignored_commands=(), travel_speed_mm_s=20.0):
        """Require unsupported commands to be explicitly delegated externally."""
        self.ignored_commands = {command.upper() for command in ignored_commands}
        self.travel_speed = travel_speed_mm_s
        self.ignored = {}
        self.position = [0.0, 0.0, 0.0]
        self.offset = [0.0, 0.0, 0.0]
        self.extrusion = 0.0
        self.xyz_absolute = True
        self.e_absolute = True
        self.feed = None
        self.motion = None
        self.unit = 1.0
        self.speed_factor = 1.0
        self.flow_factor = 1.0
        self.layer = None

    def parse(self, lines):
        """Return all segments; report failures with their original file line."""
        segments = []
        for number, raw in enumerate(lines, 1):
            try:
                segment = self.parse_line(raw, number)
            except ValueError as error:
                raise ValueError(f'G-code line {number}: {error}') from error
            if segment is not None:
                segments.append(segment)
        if not segments:
            raise ValueError('G-code contains no executable segments')
        return segments

    def parse_line(self, raw, number):
        """Accept compact words and comments; never mutate state for ignored codes."""
        layer = re.search(r';\s*LAYER:\s*(-?\d+)', raw, re.IGNORECASE)
        if layer:
            self.layer = int(layer[1])
        text = re.sub(r'\([^()]*\)', '', raw.split(';', 1)[0]).strip().upper()
        if not text:
            return None
        matches = list(WORD.finditer(text))
        if WORD.sub('', text).strip():
            raise ValueError(f'Malformed or checksummed input: {text}')
        words = {}
        for match in matches:
            letter, value = match.groups()
            if letter in words:
                raise ValueError(f'Duplicate word {letter}')
            words[letter] = None if value is None else float(value)
            if value is not None and not math.isfinite(words[letter]):
                raise ValueError(f'Nonfinite {letter}')
        words.pop('N', None)
        codes = [letter for letter in ('G', 'M', 'T') if letter in words]
        if len(codes) > 1:
            raise ValueError('Only one G/M/T command per line is supported')
        code = None
        if codes:
            letter = codes[0]
            value = words.pop(letter)
            if value is None or not value.is_integer():
                raise ValueError('Missing or fractional command number')
            code = f'{letter}{int(value)}'
        if code in self.ignored_commands:
            self.ignored.setdefault(code, []).append(number)
            return None
        if any(value is None for value in words.values()):
            raise ValueError('Missing parameter value')
        allowed = {
            'G0': 'XYZEF', 'G1': 'XYZEF', 'G4': 'PS', 'G20': '', 'G21': '',
            'G90': '', 'G91': '', 'G92': 'XYZE', 'M82': '', 'M83': '',
            'M220': 'S', 'M221': 'S', 'M400': '',
        }
        if code is None:
            if set(words) <= {'F'}:
                self.set_feed(words)
                return None
            code = self.motion
        if code not in allowed:
            raise ValueError(f'Unsupported command {code or text}')
        unexpected = set(words) - set(allowed[code])
        if unexpected:
            raise ValueError(f'Unsupported parameters for {code}: {unexpected}')
        if code in ('G90', 'G91'):
            self.xyz_absolute = self.e_absolute = code == 'G90'
        elif code in ('M82', 'M83'):
            self.e_absolute = code == 'M82'
        elif code in ('G20', 'G21'):
            self.unit = 25.4 if code == 'G20' else 1.0
        elif code == 'G92':
            if not words:
                words = dict.fromkeys('XYZE', 0.0)
            for index, axis in enumerate('XYZ'):
                if axis in words:
                    self.offset[index] = self.position[index] - words[axis] * self.unit
            if 'E' in words:
                self.extrusion = words['E'] * self.unit
        elif code in ('M220', 'M221'):
            if 'S' not in words or words['S'] < 0 or (code == 'M220' and words['S'] == 0):
                raise ValueError(f'{code} requires valid S percent (only flow may be zero)')
            if code == 'M220':
                self.speed_factor = words['S'] / 100
            else:
                self.flow_factor = words['S'] / 100
        elif code in ('G4', 'M400'):
            duration = words.get('S', words.get('P', 0.0) / 1000)
            if duration < 0:
                raise ValueError('Dwell must not be negative')
            return ToolpathSegment(tuple(self.position), tuple(self.position), 0.0,
                                   0.0, 'dwell', number, self.layer, duration)
        elif code in ('G0', 'G1'):
            self.motion = code
            return self.move(words, number, code)
        return None

    def set_feed(self, words):
        """Store a positive modal feed, converted to mm/s."""
        if 'F' in words:
            if words['F'] <= 0:
                raise ValueError('F must be positive')
            self.feed = words['F'] * self.unit / 60

    def move(self, words, number, code):
        """Resolve logical coordinates and emit a spatial or E-only segment."""
        self.set_feed(words)
        start = tuple(self.position)
        previous_e = self.extrusion
        for index, axis in enumerate('XYZ'):
            if axis in words:
                value = words[axis] * self.unit
                self.position[index] = (value + self.offset[index] if self.xyz_absolute
                                        else self.position[index] + value)
        if 'E' in words:
            value = words['E'] * self.unit
            self.extrusion = value if self.e_absolute else self.extrusion + value
        delta_e = (self.extrusion - previous_e) * self.flow_factor
        length = math.dist(start, self.position)
        if length < 1e-9 and abs(delta_e) < 1e-9:
            return None
        feed = self.feed
        if code == 'G0' and feed is None:
            feed = self.travel_speed
        if feed is None:
            raise ValueError('Motion requires a preceding or inline F feedrate')
        kind = 'travel' if delta_e == 0 else ('print' if delta_e > 0 else 'retract')
        if length < 1e-9:
            kind = 'extrude'
        return ToolpathSegment(start, tuple(self.position), delta_e,
                               feed * self.speed_factor, kind, number, self.layer)
