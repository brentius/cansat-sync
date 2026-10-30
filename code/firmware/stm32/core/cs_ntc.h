/* GPL-3.0-only. Twin of sim/temperature/models.py code_volts,
 * resistance_from_volts and sh/beta_temperature. Default SH fit is nominal,
 * NOT calibration; beta is optional. PGA follows ADS1115 bits 11:9 (0..7).
 * Duty guard is a no-burst token bucket: each accepted pulse reserves ON time
 * and blocks until on_us / ceiling elapsed. Caller MUST turn excitation off
 * by the reserved end and call finish; overruns are counted, block future ON.
 * A 4634 us pair at 10 Hz satisfies the [assumed] <=6% candidate ceiling. */
#ifndef CS_NTC_H
#define CS_NTC_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { CS_NTC_OK=0, CS_NTC_SHORT, CS_NTC_OPEN, CS_NTC_SATURATED, CS_NTC_CONFIG, CS_NTC_DOMAIN } cs_ntc_fault_t;
typedef struct {
    float r_ref, r25, beta, sh_a, sh_b, sh_c;
    float min_ohm, max_ohm; bool use_sh;
} cs_ntc_config_t;
typedef struct { float v_ntc, v_exc, resistance; int16_t temperature_centi_c; cs_ntc_fault_t fault; } cs_ntc_result_t;
typedef struct { uint64_t next_us, started_us, reserved_us, total_on_us; float ceiling; bool active; uint32_t denied, overruns; } cs_ntc_duty_t;
cs_ntc_config_t cs_ntc_default_config(void);
float cs_ntc_code_volts(int16_t code, unsigned pga);
cs_ntc_fault_t cs_ntc_convert(const cs_ntc_config_t *cfg, int16_t ain0, unsigned pga0,
                             int16_t ain1, unsigned pga1, cs_ntc_result_t *out);
bool cs_ntc_duty_init(cs_ntc_duty_t *duty, float ceiling);
bool cs_ntc_duty_begin(cs_ntc_duty_t *duty, uint64_t now_us, uint32_t on_us);
void cs_ntc_duty_finish(cs_ntc_duty_t *duty, uint64_t now_us);
#endif
