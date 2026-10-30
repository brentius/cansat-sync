/* GPL-3.0-only. C twin of cansat/downlink.py DownlinkScheduler. */
#ifndef CS_DOWNLINK_H
#define CS_DOWNLINK_H
#include "cs_lora.h"
#include "cs_flight_state.h"
typedef struct { cs_duty_t limiter; double airtime, flight_interval, reserve, ground_interval, last; } cs_downlink_t;
bool cs_downlink_init(cs_downlink_t *s, double airtime_s, double duty_limit, double window_s,
                      double flight_interval_s, double flight_reserve_s, double ground_interval_s);
/* True commits budget: caller must transmit exactly one packet. */
bool cs_downlink_due(cs_downlink_t *s, double now, cs_flight_state_t state);
#endif
