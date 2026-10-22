"""GPL-3.0-only. System sizing models: canopy, descent, energy and exposure.

Standard library only. These are design-stage estimates. Every numerical input
supplied by run_sizing.py is labelled there as a rule, a sourced figure or an
assumption; nothing here is a measurement of the CanSat.
"""
import math

G0 = 9.80665
R_DRY = 287.05


def air_density(pressure_pa, temperature_k):
    return pressure_pa/(R_DRY*temperature_k)


# --- Canopy -----------------------------------------------------------------

def drag_area(mass_kg, speed, rho):
    """Cd*S that balances weight at a steady descent speed."""
    return 2*mass_kg*G0/(rho*speed**2)


def descent_speed(mass_kg, rho, cd_s):
    return math.sqrt(2*mass_kg*G0/(rho*cd_s))


def nominal_diameter(cd_s, cd):
    """Diameter of the circle whose area is the nominal canopy area S0."""
    return math.sqrt(4*cd_s/(cd*math.pi))


def fall_distance(terminal_speed, time_s):
    """Distance fallen from rest with quadratic drag and the canopy open."""
    return terminal_speed**2/G0*math.log(math.cosh(G0*time_s/terminal_speed))


def distance_to_fraction(terminal_speed, fraction):
    """Fall distance from rest to reach a fraction of terminal speed."""
    return -terminal_speed**2/(2*G0)*math.log(1 - fraction**2)


def terminal_speed_from_drop(height_m, time_s):
    """Inverts fall_distance for terminal speed; None if faster than free fall.

    Height divided by time underestimates terminal speed because it averages
    over the acceleration from rest. This removes that bias for a canopy that
    is open at release; inflation time is not modelled.
    """
    if height_m >= 0.5*G0*time_s**2:
        return None
    low, high = height_m/time_s, 1e4
    for _ in range(200):
        mid = (low + high)/2
        if fall_distance(mid, time_s) < height_m:
            low = mid
        else:
            high = mid
    return (low + high)/2


def opening_force(cd_s, rho, speed, cx=1.7, x1=1.0):
    """Peak opening force, Knacke form F = CdS q Cx X1.

    x1 = 1 is the infinite-mass bound; a light payload decelerates during
    inflation and sees less.
    """
    return cd_s*0.5*rho*speed**2*cx*x1


def speed_for_opening_force(force_n, cd_s, rho, cx=1.7, x1=1.0):
    return math.sqrt(2*force_n/(rho*cd_s*cx*x1))


def target_speed_for_floor(floor_speed, rho_test, rho_max, relative_sigma, k=2.0):
    """Speed to measure at test density so that the densest launch-day air and
    a k-sigma measurement error still leave the descent above the floor."""
    return floor_speed*math.sqrt(rho_max/rho_test)*(1 + k*relative_sigma)


def drop_test_relative_sigma(height_m, time_s, sigma_height, sigma_time):
    """First-order uncertainty of a terminal speed inferred from one drop."""
    return math.hypot(sigma_height/height_m, sigma_time/time_s)


# --- Descent timeline -------------------------------------------------------

def descent_timeline(release_height, speed, frame_rate=5.0, map_floor=80.0, layer=20.0):
    lag = speed**2/G0
    return {
        'time_s': release_height/speed,
        'frames_total': release_height/speed*frame_rate,
        'frames_above_map_floor': max(release_height - map_floor, 0)/speed*frame_rate,
        'frames_per_layer': layer/speed*frame_rate,
        'lag_length_m': lag,
        'lag_lengths_per_layer': layer/lag,
    }


# --- Energy -----------------------------------------------------------------

def battery_energy_wh(capacity_mah, nominal_v=3.6, usable_fraction=0.85):
    return capacity_mah/1000*nominal_v*usable_fraction


def endurance_hours(usable_wh, load_w, converter_efficiency):
    return usable_wh*converter_efficiency/load_w


def required_capacity_mah(load_w, hours, converter_efficiency, nominal_v=3.6, usable_fraction=0.85):
    return load_w*hours/converter_efficiency/(nominal_v*usable_fraction)*1000


# --- Exposure ---------------------------------------------------------------

def photoelectrons(scene_lux, reflectance, f_number, exposure_s, pixel_m=3e-6,
                   transmission=0.85, quantum_efficiency=0.5, photons_per_lux=1.0e16):
    """Mean signal per pixel for a Lambertian scene at infinity.

    photons_per_lux is photons per square metre per second per lux. 4.1e15 is
    exact for 555 nm; daylight seen by a silicon sensor is higher. 1e16 is an
    assumed round figure, uncertain by about a factor of two.
    """
    image_lux = scene_lux*reflectance*transmission/(4*f_number**2)
    return image_lux*photons_per_lux*pixel_m**2*exposure_s*quantum_efficiency


def shot_noise_snr(electrons, read_noise=5.0):
    return electrons/math.sqrt(electrons + read_noise**2)
