"""GPL-3.0-only. Dry-air relations used by flight software and sizing tools."""
import math

G0 = 9.80665
R_DRY = 287.05


def air_density(pressure_pa, temperature_k):
    return pressure_pa / (R_DRY*temperature_k)


def pressure_altitude(pressure_pa, reference_pa, mean_temperature_k):
    """Hypsometric height of a pressure level above the reference level."""
    return R_DRY*mean_temperature_k/G0*math.log(reference_pa/pressure_pa)


def pressure_at(height_m, reference_pa, mean_temperature_k):
    return reference_pa*math.exp(-G0*height_m/(R_DRY*mean_temperature_k))
