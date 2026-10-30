"""GPL-3.0-only. Image correction against analytic grid and intensity ramps."""
import numpy as np
import pytest
from cansat.eskf import Camera
from cansat.rectify import Rectifier


def test_identity_preserves_owned_raw_array_without_copy():
    image = np.arange(81, dtype=np.uint16).reshape(9, 9)
    assert Rectifier(Camera(4., 4., 4., 4., 9, 9)).apply(image) is image


def test_radial_correction_samples_the_raw_intensity_at_distorted_ray():
    # Target ray x=0.5, y=0 has raw x=0.5*(1+0.4*0.25)=0.55.
    # fx=20 means raw column 31 instead of target column 30.
    camera = Camera(20., 20., 20., 20., 41, 41)
    image = np.broadcast_to(np.arange(41, dtype=np.uint16)*100, (41, 41)).copy()
    result = Rectifier(camera, (.4, 0, 0, 0, 0)).apply(image)
    assert result[20, 30] == 3100
    assert result[20, 20] == 2000
    assert result[20, 40] == 0  # outside sensor, never wrap around
    np.testing.assert_array_equal(image[20], np.arange(41)*100)


def test_tangential_correction_and_strip_boundary():
    camera = Camera(20., 20., 20., 40., 41, 81)
    image = np.broadcast_to(np.arange(81, dtype=np.uint16)[:, None]*100, (81, 41)).copy()
    result = Rectifier(camera, (0, 0, .1, 0, 0)).apply(image)
    # y=0 at target (30,40), x=.5 => raw y=.1*.25=.025 -> row 40.5.
    assert result[40, 30] == 4050
    assert result[32, 20] > 3200


@pytest.mark.parametrize('coeffs', [(0, 0), (0, 0, 0, 0, float('nan'))])
def test_malformed_model_refused(coeffs):
    with pytest.raises(ValueError):
        Rectifier(Camera(1., 1., 1., 1., 3, 3), coeffs)
