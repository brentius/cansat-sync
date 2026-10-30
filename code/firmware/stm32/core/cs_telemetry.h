/* GPL-3.0-only. C twin of telemetry.encode/decode and link.apply_nav.
 * Struct contains wire-coded values (names match telemetry.Telemetry).
 * Encoding uses those integers directly; SI conversion helpers use Python's
 * ties-to-even rounding, unlike link's floor(x+0.5). NA is INT16_MIN,
 * UINT8_MAX or UINT16_MAX. Pressure is Pa-50000; battery is (V-2.5)/0.01.
 */
#ifndef CS_TELEMETRY_H
#define CS_TELEMETRY_H
#include "cs_link.h"
#define CS_TELEMETRY_SIZE 40u
#define CS_TELEMETRY_MAGIC 0xc5u
#define CS_FLAG_GPS_VALID 0x10u
typedef struct {
    uint16_t seq; uint32_t t_ms; uint8_t state, flags;
    uint16_t pressure_pa;
    int16_t temperature_c, altitude_m, gps_e, gps_n, gps_u, trn_e, trn_n;
    uint8_t trn_major_m, trn_minor_m, trn_angle_rad, protection_m, landmarks;
    int16_t pred_e, pred_n;
    uint8_t time_to_go_s, pred_major_m, pred_minor_m, pred_angle_rad, battery_v;
} cs_telemetry_t;
/* Initialise all unavailable fields to NA; angles/landmarks/state/counters to 0. */
void cs_telemetry_init(cs_telemetry_t *packet);
size_t cs_telemetry_encode(uint8_t *dst, size_t cap, const cs_telemetry_t *packet);
bool cs_telemetry_decode(cs_telemetry_t *packet, const uint8_t *src, size_t len);
int16_t cs_telemetry_i16(double value, double step);
uint8_t cs_telemetry_u8(double value, double step, double offset);
uint16_t cs_telemetry_u16(double value, double step, double offset);
uint8_t cs_telemetry_angle(double radians);
/* Mutates only navigation fields/flags; negative max_age_s disables age check.
 * Age is unsigned modular microseconds; exact threshold is accepted. NULL nav
 * or stale nav clears fields to NA and camera/TRN/pred flags. */
void cs_apply_nav(cs_telemetry_t *packet, const cs_link_nav_t *nav, double max_age_s, uint32_t now_tick);
#endif
