/* GPL-3.0-only. C twin of cansat/lora.py time_on_air and DutyCycleLimiter.
 * Monotonic seconds. Ring full fails closed, never forgets unexpired airtime.
 * 4096 records cover >=1 packet/s over the default 3600 s window. */
#ifndef CS_LORA_H
#define CS_LORA_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define CS_DUTY_CAP 4096u
typedef struct { unsigned sf, coding_rate, preamble; double bw_hz; bool crc, implicit_header; int ldro; } cs_lora_config_t;
typedef struct {
    double duty_limit, window_s, used_s, last_time;
    double time[CS_DUTY_CAP], airtime[CS_DUTY_CAP];
    size_t head, count;
    uint32_t rejected;
} cs_duty_t;
cs_lora_config_t cs_lora_default_config(void);
double cs_lora_time_on_air(size_t payload_bytes, const cs_lora_config_t *config);
bool cs_duty_init(cs_duty_t *duty, double limit, double window_s);
double cs_duty_used(cs_duty_t *duty, double now);
double cs_duty_remaining(cs_duty_t *duty, double now);
bool cs_duty_allowed(cs_duty_t *duty, double now, double airtime, double reserve);
bool cs_duty_record(cs_duty_t *duty, double now, double airtime);
#endif
