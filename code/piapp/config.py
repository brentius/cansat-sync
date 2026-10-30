"""GPL-3.0-only. Strict TOML configuration for the PI-SHELL interface contract.

Scheduling, queue sizes and latency defaults are [assumed], not Pi measurements.
"""
from dataclasses import dataclass, field, fields
import math
from pathlib import Path
import tomllib


def _number(value, lo, hi, name, integer=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not lo <= value <= hi
            or (integer and not isinstance(value, int))):
        raise ValueError(f'invalid {name}: expected {lo}..{hi}')


@dataclass(frozen=True)
class CameraConfig:
    kind: str = 'picamera2'
    width: int = 1456
    height: int = 1088
    gain: float = 1.
    exposure_us: int = 500
    period_s: float = .2
    replay_index: Path | None = None
    trigger_mode_path: str = '/sys/module/imx296/parameters/trigger_mode'

    def __post_init__(self):
        if self.kind not in ('picamera2', 'synthetic', 'replay'):
            raise ValueError('camera.kind must be picamera2, synthetic or replay')
        _number(self.width, 2, 1456, 'camera.width', True)
        _number(self.height, 2, 1088, 'camera.height', True)
        if self.width % 2 or self.height % 2:
            raise ValueError('camera dimensions must be even')
        _number(self.gain, 1, 16, 'camera.gain')
        _number(self.exposure_us, 1, 65000, 'camera.exposure_us', True)
        _number(self.period_s, .05, 60, 'camera.period_s')
        if self.kind == 'replay' and self.replay_index is None:
            raise ValueError('replay requires camera.replay_index')
        if not isinstance(self.trigger_mode_path, str) or not self.trigger_mode_path:
            raise ValueError('camera.trigger_mode_path required')


@dataclass(frozen=True)
class Config:
    port: str = '/dev/ttyAMA0'
    baud: int = 230400
    site: Path | None = None
    data_dir: Path = Path('/data/cansat')
    message_capacity: int = 1500
    writer_capacity: int = 32
    capture_on_pad: bool = False
    landed_capture_s: float = 5.
    trigger_period_ms: int = 200
    trigger_pulse_us: int = 486
    frame_latency_s: float = .020
    frame_wait_s: float = .1
    max_trigger_age_s: float = 1.
    max_frame_age_s: float = 8.
    max_nav_cache_age_s: float = .6
    sync_interval_s: float = 1.
    shutdown_timeout_s: float = 15.
    poll_interval_s: float = .005
    allow_poweroff: bool = False
    camera: CameraConfig = field(default_factory=CameraConfig)
    navigator: dict = field(default_factory=dict)

    def __post_init__(self):
        for name, low, high in (('baud', 9600, 921600), ('message_capacity', 2, 20000),
                                ('writer_capacity', 1, 128), ('trigger_period_ms', 50, 60000),
                                ('trigger_pulse_us', 1, 65000)):
            _number(getattr(self, name), low, high, name, True)
        for name, low, high in (('landed_capture_s', 0, 3600), ('frame_latency_s', 0, 10),
                                ('frame_wait_s', 0, 1), ('max_trigger_age_s', .1, 30),
                                ('max_frame_age_s', .1, 30), ('max_nav_cache_age_s', .01, 2),
                                ('sync_interval_s', .01, 60), ('shutdown_timeout_s', .1, 120),
                                ('poll_interval_s', .001, .1)):
            _number(getattr(self, name), low, high, name)
        if self.trigger_pulse_us + 15 >= self.trigger_period_ms * 1000:
            raise ValueError('trigger pulse must fit within period')
        if not isinstance(self.port, str) or not self.port:
            raise ValueError('port required')
        if type(self.capture_on_pad) is not bool or type(self.allow_poweroff) is not bool:
            raise ValueError('capture_on_pad and allow_poweroff must be booleans')
        if not isinstance(self.camera, CameraConfig) or not isinstance(self.navigator, dict):
            raise ValueError('invalid camera/navigator configuration')


def _construct(cls, values):
    if not isinstance(values, dict):
        raise ValueError(f'{cls.__name__} must be a table')
    unknown = values.keys() - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f'unknown {cls.__name__} fields: {sorted(unknown)}')
    return cls(**values)


def load(path):
    path = Path(path).resolve()
    with path.open('rb') as stream:
        values = tomllib.load(stream)
    if values.keys() - {'service', 'camera', 'navigator'}:
        raise ValueError('only service, camera and navigator tables are supported')
    service = dict(values.get('service', {}))
    camera = dict(values.get('camera', {}))
    for table, names in ((service, ('site', 'data_dir')), (camera, ('replay_index',))):
        for name in names:
            if name in table:
                value = table[name]
                if not isinstance(value, str) or not value:
                    raise ValueError(f'{name} must be a nonempty path')
                table[name] = (path.parent / value).resolve()
    if service.keys() & {'camera', 'navigator'}:
        raise ValueError('camera/navigator must be separate tables')
    return _construct(Config, {**service, 'camera': _construct(CameraConfig, camera),
                               'navigator': values.get('navigator', {})})
