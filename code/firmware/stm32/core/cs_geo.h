/* GPL-3.0-only. Exact C twin of cansat/geo.py enu_from_link; double needed
 * for subtraction of Earth-scale ECEF coordinates. Heights are hMSL on both
 * sides, as on the link (not a geoid correction). Output east,north,up metres. */
#ifndef CS_GEO_H
#define CS_GEO_H
#include <stdbool.h>
#include <stdint.h>
bool cs_geo_enu_from_link(int32_t lat, int32_t lon, int32_t h,
                          int32_t lat0, int32_t lon0, int32_t h0, double enu[3]);
#endif
