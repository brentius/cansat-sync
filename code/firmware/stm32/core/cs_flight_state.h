/* GPL-3.0-only. C twin of cansat/flight_state.py. Times are monotonic seconds.
 * Maximum update rate 100 Hz, including calls without a valid pressure sample.
 * Capacities cover defaults (3 s delay, .25 s median, 1 s speed) at that rate.
 * Init rejects larger windows. Excess rate evicts oldest samples and counts
 * overflow; equivalence to Python is guaranteed only within these bounds.
 * Missing sensors are NAN. Restore discards histories/holds; entry times and
 * baseline are supplied in the NEW clock domain by the persistence owner. */
#ifndef CS_FLIGHT_STATE_H
#define CS_FLIGHT_STATE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef enum { CS_PAD=0, CS_ASCENT=1, CS_DESCENT=2, CS_LANDED=3 } cs_flight_state_t;
typedef enum { CS_REASON_NONE=0, CS_REASON_ACCELERATION, CS_REASON_ALTITUDE, CS_REASON_TIMEOUT } cs_fsm_reason_t;
typedef struct {
    double launch_accel, launch_hold_s, launch_alt_m, launch_climb_rate, launch_alt_hold_s;
    double baseline_tau_s, baseline_delay_s, median_window_s, speed_window_s, mean_temp_k;
    double eject_lockout_s, eject_accel, apogee_drop_m, apogee_hold_s, ascent_timeout_s;
    double touchdown_lockout_s, touchdown_accel, touchdown_alt_m, rest_speed, rest_hold_s, descent_timeout_s;
} cs_fsm_config_t;
typedef struct { double t, value; } cs_fsm_sample_t;
#define CS_FSM_PENDING_CAP 302u
#define CS_FSM_MEDIAN_CAP 27u
#define CS_FSM_HISTORY_CAP 102u
typedef struct {
    cs_fsm_config_t cfg;
    cs_flight_state_t state;
    cs_fsm_reason_t reason; /* last transition; changed only on an event */
    double baseline_pa, altitude, vertical_speed, max_altitude;
    double t_launch, t_descent, t_landed, baseline_t, last_t;
    double hold_since[4];
    cs_fsm_sample_t pending[CS_FSM_PENDING_CAP], recent[CS_FSM_MEDIAN_CAP], history[CS_FSM_HISTORY_CAP];
    size_t npending, nrecent, nhistory;
    uint32_t events, overflow, invalid_time;
} cs_fsm_t;
cs_fsm_config_t cs_fsm_default_config(void);
bool cs_fsm_init(cs_fsm_t *fsm, const cs_fsm_config_t *config);
bool cs_fsm_restore(cs_fsm_t *fsm, cs_flight_state_t state, double baseline_pa,
                    double max_altitude, double t_launch, double t_descent, double t_landed);
cs_flight_state_t cs_fsm_update(cs_fsm_t *fsm, double t, double accel, double pressure_pa);
#endif
