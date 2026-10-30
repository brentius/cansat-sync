"""GPL-3.0-only. Prepared map/camera/wind bundle for Pi navigation.

Implements the PI-NAV site contract; products are those of coarse.CoarseMap,
fine.FineMatcher and landing.DriftTable. Raster row=north, column=east, z=0
is the pad ground plane. Heights in pad_origin and GNSS must share the stated
datum. NPZ contains numeric arrays plus UTF-8 JSON metadata, never pickle.
Real orthophoto preparation, distortion calibration and datum survey are open.
"""
from dataclasses import asdict, dataclass, field
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

from .coarse import CoarseConfig, CoarseMap
from .eskf import Camera
from .fine import FineConfig, FineMatcher
from .landing import DriftTable
from .windprofile import N_AXIS, WindModel


@dataclass
class Site:
    raster: np.ndarray
    resolution_m: float
    raster_origin: tuple
    pad_origin: tuple
    datum: str
    camera: Camera
    centre: tuple = (0., 0.)
    coarse_n1: int = 256
    coarse_config: CoarseConfig = field(default_factory=CoarseConfig)
    fine_config: FineConfig = field(default_factory=FineConfig)
    wind_model: WindModel = field(default_factory=lambda: WindModel(max_substeps=32))
    descent_speed_mps: float = 10.5  # [assumed] replace with drop-test calibration
    table_top_m: float = 450.
    table_step_m: float = 2.
    drift_table: DriftTable | None = None
    distortion: tuple = (0., 0., 0., 0., 0.)  # calibrated k1,k2,p1,p2,k3, or already-rectified input

    def __post_init__(self):
        self.raster = np.asarray(self.raster, dtype=float)
        self.raster_origin, self.pad_origin, self.centre = (
            tuple(v) for v in (self.raster_origin, self.pad_origin, self.centre))
        if (self.raster.ndim != 2 or min(self.raster.shape) < 16
                or not np.isfinite(self.raster).all()):
            raise ValueError('finite 2-D raster required')
        if (len(self.raster_origin) != 2 or len(self.pad_origin) != 3 or len(self.centre) != 2
                or not np.isfinite((*self.raster_origin, *self.pad_origin, *self.centre)).all()
                or abs(self.pad_origin[0]) > 90 or abs(self.pad_origin[1]) > 180
                or not isinstance(self.datum, str) or not self.datum.strip()):
            raise ValueError('finite origins and explicit datum notes required')
        c = self.camera
        self.distortion = tuple(self.distortion)
        if len(self.distortion) != 5 or not np.isfinite(self.distortion).all():
            raise ValueError('five finite calibrated distortion coefficients required')
        if (not np.isfinite((c.fx, c.fy, c.cx, c.cy)).all() or min(c.fx, c.fy, c.width, c.height) <= 0
                or np.shape(c.r_cb) != (3, 3) or np.shape(c.lever) != (3,)
                or not np.isfinite(c.lever).all()
                or not np.allclose(c.r_cb @ c.r_cb.T, np.eye(3), atol=1e-6)
                or not math.isclose(float(np.linalg.det(c.r_cb)), 1., abs_tol=1e-6)):
            raise ValueError('calibrated finite pinhole camera/extrinsics required')
        if (not np.isfinite(self.resolution_m) or self.resolution_m <= 0
                or not isinstance(self.coarse_n1, int) or not 16 <= self.coarse_n1 <= 512):
            raise ValueError('positive resolution and bounded coarse map size required')
        if not all(math.isfinite(v) and v > 0 for v in (
                self.descent_speed_mps, self.table_top_m, self.table_step_m)):
            raise ValueError('positive finite drift table settings required')
        if not 2 <= math.ceil(self.table_top_m/self.table_step_m) <= 10000:
            raise ValueError('drift table must have 2..10000 intervals')
        self.coarse_map = CoarseMap(self.raster, self.resolution_m, self.raster_origin,
                                    self.centre, self.coarse_n1, self.coarse_config)
        self.fine_matcher = FineMatcher(self.raster, self.resolution_m, self.raster_origin,
                                        self.fine_config)
        if self.drift_table is None:
            self.drift_table = DriftTable(self.wind_model, self.descent_speed_mps,
                                          self.table_top_m, self.table_step_m)


def save(path, site):
    """Write a self-contained NPZ with JSON metadata and prepared drift arrays.

    Map features/templates are deterministically rebuilt on load before flight.
    The caller owns path replacement policy (no raw logs are touched here).
    """
    metadata = {name: getattr(site, name) for name in (
        'resolution_m', 'raster_origin', 'pad_origin', 'datum', 'centre', 'coarse_n1',
        'descent_speed_mps', 'table_top_m', 'table_step_m', 'distortion')}
    metadata.update(schema_version=1, coarse_config=asdict(site.coarse_config),
                    fine_config=asdict(site.fine_config), wind_model=asdict(site.wind_model),
                    camera={key: getattr(site.camera, key) for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height')})
    with Path(path).open('wb') as stream:
        np.savez_compressed(stream, metadata=json.dumps(metadata, allow_nan=False), raster=site.raster,
                            r_cb=site.camera.r_cb, lever=site.camera.lever,
                            drift_heights=site.drift_table.heights, drift_rows=site.drift_table.rows,
                            drift_noise=site.drift_table.noise)


def load(path):
    """Load an allow_pickle=False bundle; reject incompatible or corrupt data."""
    with np.load(path, allow_pickle=False) as data:
        meta = json.loads(str(data['metadata']))
        if meta.pop('schema_version', None) != 1:
            raise ValueError('unsupported site schema')
        cc, fc = meta.pop('coarse_config'), meta.pop('fine_config')
        for name in ('sites', 'polish_sites'):
            cc[name] = tuple(cc[name])
        fc['cells'] = tuple(fc['cells'])
        camera = Camera(**meta.pop('camera'), r_cb=data['r_cb'].copy(), lever=data['lever'].copy())
        table = object.__new__(DriftTable)
        table.descent_speed = float(meta['descent_speed_mps'])
        table.heights, table.rows, table.noise = (data[key].copy() for key in (
            'drift_heights', 'drift_rows', 'drift_noise'))
        heights = table.heights
        if (heights.ndim != 1 or not 3 <= len(heights) <= 10001
                or table.rows.shape != (len(heights), N_AXIS) or table.noise.shape != heights.shape
                or not all(np.isfinite(a).all() for a in (heights, table.rows, table.noise))
                or heights[0] != 0 or not np.allclose(np.diff(heights), meta['table_step_m'])
                or np.any(table.noise < 0)):
            raise ValueError('corrupt drift table')
        return Site(raster=data['raster'].copy(), camera=camera,
                    coarse_config=CoarseConfig(**cc), fine_config=FineConfig(**fc),
                    wind_model=WindModel(**meta.pop('wind_model')), drift_table=table, **meta)


def from_farmland(seed=20261026, *, size_m=1280., resolution_m=1., style='mixed',
                  pad_origin=(22., 114., 15.), camera=None, **kwargs):
    """Bench/test builder using sim/nav/farmland.py, imported only on request.

    Returns a Site, not flight truth. Default IMX296 intrinsics/lever are
    [assumed] candidate values reused from run_fine_mc, not a calibration.
    Procedural map height variations are deliberately ignored by planar TRN.
    """
    path = Path(__file__).resolve().parents[2] / 'sim' / 'nav' / 'farmland.py'
    spec = importlib.util.spec_from_file_location('_cansat_bench_farmland', path)
    if spec is None or spec.loader is None:
        raise RuntimeError('procedural terrain builder is unavailable')
    import sys
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    world = module.generate(seed, size_m=size_m, res_m=resolution_m, style=style)
    camera = camera or Camera(724.6, 724.6, 728., 544., 1456, 1088,
                              lever=np.array([0., 0., .045]))
    return Site(world.map, world.res, world.origin, pad_origin,
                'SYNTHETIC procedural farmland; assumed MSL origin; flat z=0 pad plane', camera, **kwargs)
