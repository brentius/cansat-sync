"""GPL-3.0-only. WGS84 geodetic to local east/north/up about the launch-pad origin.

Both processors put GNSS fixes in the same pad frame: the STM32 fills the
packet's gps e/n/u from the origin it holds (Status message), and the Pi uses
that origin to compare GNSS with TRN. This module is the reference the STM32
firmware's conversion must match. It is exact (geodetic -> ECEF -> rotation
into the origin's tangent plane), not a flat-earth approximation; the two
differ by centimetres over the few kilometres a CanSat drifts, but matching
the reference exactly keeps the two processors' numbers identical.

Heights are ellipsoidal or MSL consistently on both sides; the link carries
hMSL, so the frame's up axis is relative to the origin's hMSL.
"""
import math

A = 6378137.0
F = 1/298.257223563
E2 = F*(2 - F)


def ecef(lat_deg, lon_deg, h_m):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    s, c = math.sin(lat), math.cos(lat)
    n = A/math.sqrt(1 - E2*s*s)
    return ((n + h_m)*c*math.cos(lon), (n + h_m)*c*math.sin(lon), (n*(1 - E2) + h_m)*s)


def enu(lat_deg, lon_deg, h_m, lat0_deg, lon0_deg, h0_m):
    """East, north, up metres of a point from the origin."""
    x, y, z = ecef(lat_deg, lon_deg, h_m)
    x0, y0, z0 = ecef(lat0_deg, lon0_deg, h0_m)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat0, lon0 = math.radians(lat0_deg), math.radians(lon0_deg)
    sl, cl = math.sin(lat0), math.cos(lat0)
    so, co = math.sin(lon0), math.cos(lon0)
    e = -so*dx + co*dy
    n = -sl*co*dx - sl*so*dy + cl*dz
    u = cl*co*dx + cl*so*dy + sl*dz
    return e, n, u


def enu_from_link(lat_e7, lon_e7, h_mm, lat0_e7, lon0_e7, h0_mm):
    """Same, from the link's integer units (1e-7 deg, mm)."""
    return enu(lat_e7*1e-7, lon_e7*1e-7, h_mm*1e-3, lat0_e7*1e-7, lon0_e7*1e-7, h0_mm*1e-3)
