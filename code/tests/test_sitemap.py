"""GPL-3.0-only. Prepared site persistence and coordinate-contract tests."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat import sitemap
from cansat.coarse import CoarseConfig
from cansat.eskf import Camera
from cansat.windprofile import WindModel


def small_site():
    return sitemap.Site(
        raster=np.random.default_rng(8).uniform(0, 255, (256, 256)),
        resolution_m=1., raster_origin=(-128., -128.),
        pad_origin=(22., 114., 15.), datum='synthetic MSL; flat ground at pad',
        camera=Camera(100., 100., 79.5, 59.5, 160, 120),
        coarse_n1=64, coarse_config=CoarseConfig(),
        wind_model=WindModel(max_substeps=8), table_top_m=40., table_step_m=10.)


def test_roundtrip_preserves_products_and_json_metadata(tmp_path):
    site = small_site()
    site.distortion = (-.1, .02, .001, -.002, .003)
    path = tmp_path / 'site.npz'
    sitemap.save(path, site)
    other = sitemap.load(path)
    assert other.pad_origin == site.pad_origin
    assert other.datum == site.datum
    assert other.distortion == site.distortion
    np.testing.assert_array_equal(other.raster, site.raster)
    np.testing.assert_array_equal(other.coarse_map.f2, site.coarse_map.f2)
    np.testing.assert_array_equal(other.fine_matcher.points, site.fine_matcher.points)
    np.testing.assert_array_equal(other.drift_table.rows, site.drift_table.rows)
    np.testing.assert_array_equal(other.camera.r_cb, site.camera.r_cb)
    with np.load(path, allow_pickle=False) as bundle:
        assert 'metadata' in bundle


@pytest.mark.parametrize('change', [dict(raster_origin=(float('nan'), 0)),
                                   dict(pad_origin=(91., 0., 0.)),
                                   dict(datum=''), dict(resolution_m=-1.)])
def test_invalid_site_refused(change):
    site = small_site()
    values = dict(raster=site.raster, resolution_m=1., raster_origin=(-128., -128.),
                  pad_origin=(22., 114., 15.), datum='test', camera=site.camera, coarse_n1=64)
    values.update(change)
    with pytest.raises(ValueError):
        sitemap.Site(**values)


def test_unknown_schema_is_refused(tmp_path):
    path = tmp_path / 'bad.npz'
    np.savez(path, metadata='{"schema_version":999}')
    with pytest.raises(ValueError, match='schema'):
        sitemap.load(path)


def test_corrupt_drift_table_is_refused_without_pickle(tmp_path):
    path = tmp_path / 'site.npz'
    sitemap.save(path, small_site())
    with np.load(path, allow_pickle=False) as data:
        arrays = {key: data[key].copy() for key in data.files}
    arrays['drift_noise'][1] = -1.
    np.savez(path, **arrays)
    with pytest.raises(ValueError, match='drift table'):
        sitemap.load(path)
