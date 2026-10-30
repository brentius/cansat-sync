/* GPL-3.0-only. */
#include "cs_atmos.h"
/* Bare Zig supplies no target libc headers; final firmware supplies these
 * standard C99 memory/libm symbols. Host builds use their system headers. */
#if __STDC_HOSTED__
#include <math.h>
#else
extern double log(double);
extern double exp(double);
#endif
double cs_air_density(double p, double t) { return p/(CS_R_DRY*t); }
double cs_pressure_altitude(double p, double p0, double t) { return CS_R_DRY*t/CS_G0*log(p0/p); }
double cs_pressure_at(double h, double p0, double t) { return p0*exp(-CS_G0*h/(CS_R_DRY*t)); }
