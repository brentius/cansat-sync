/* GPL-3.0-only. C twin of cansat/atmosphere.py, SI units. */
#ifndef CS_ATMOS_H
#define CS_ATMOS_H
#define CS_G0 9.80665
#define CS_R_DRY 287.05
double cs_air_density(double pressure_pa, double temperature_k);
double cs_pressure_altitude(double pressure_pa, double reference_pa, double temperature_k);
double cs_pressure_at(double height_m, double reference_pa, double temperature_k);
#endif
