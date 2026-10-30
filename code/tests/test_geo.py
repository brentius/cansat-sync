"""GPL-3.0-only."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from cansat import geo


def test_origin_is_zero():
    assert geo.enu(51.5, -1.2, 100.0, 51.5, -1.2, 100.0) == (0.0, 0.0, 0.0)


def test_small_offsets_match_local_radii():
    lat0 = 52.0
    m = geo.A*(1 - geo.E2)/(1 - geo.E2*math.sin(math.radians(lat0))**2)**1.5
    e, n, u = geo.enu(lat0 + 1e-4, 0.0, 0.0, lat0, 0.0, 0.0)
    assert abs(n - m*math.radians(1e-4)) < 1e-3 and abs(e) < 1e-9 and abs(u) < 1e-3
    e, n, u = geo.enu(lat0, 1e-4, 0.0, lat0, 0.0, 0.0)
    nr = geo.A/math.sqrt(1 - geo.E2*math.sin(math.radians(lat0))**2)
    assert abs(e - nr*math.cos(math.radians(lat0))*math.radians(1e-4)) < 1e-3


def test_up_and_link_units():
    e, n, u = geo.enu_from_link(520000000, -12000000, 400000, 520000000, -12000000, 100000)
    assert abs(u - 300.0) < 1e-6 and abs(e) < 1e-6 and abs(n) < 1e-6
