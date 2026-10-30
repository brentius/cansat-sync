/* GPL-3.0-only. Pure state machine for stm32_pi_link_protocol.md section 8.
 * now_ms is monotonic modulo uint32 (poll gaps/durations <2^31 ms).
 * Outputs are desired levels; send flags are one-poll events. Caller sends
 * Hello contents itself; shutdown outputs a wire-coded cs_link_cmd_t.
 * request_shutdown is explicit policy input (not automatic at LANDED).
 * STOPPED stays off. Default policy keeps power on through silence at all
 * flight stages; alive still expires independently. cycle_silence_ms=0 disables
 * automatic recovery. A nonzero value opts into PAD silence cycling and at
 * most one in-flight recovery if silent at launch (optional bench policy). */
#ifndef CS_PIMGR_H
#define CS_PIMGR_H
#include "cs_flight_state.h"
#include "cs_link.h"
typedef struct { uint32_t settle_ms, alive_ms, cycle_silence_ms, power_off_ms, cmd_period_ms, ack_settle_ms, shutdown_timeout_ms; } cs_pimgr_config_t;
typedef enum { CS_PI_WAIT_POWER=0, CS_PI_SETTLE, CS_PI_RUNNING, CS_PI_CYCLE_OFF, CS_PI_SHUTTING, CS_PI_STOPPED } cs_pi_phase_t;
typedef struct { uint32_t now_ms; cs_flight_state_t flight; bool pgood, rx_valid, request_shutdown; const cs_link_ack_t *ack; } cs_pimgr_input_t;
typedef struct { bool oe_high, pwr_en, alive, stop_triggers, send_hello, send_cmd; cs_link_cmd_t cmd; } cs_pimgr_output_t;
typedef struct {
    cs_pimgr_config_t cfg; cs_pi_phase_t phase; cs_flight_state_t last_flight;
    uint32_t phase_ms, last_rx_ms, last_cmd_ms, done_ms;
    uint16_t cmd_seq; bool seen_rx, acked, done, sent_cmd, launch_silent, flight_cycle_used;
} cs_pimgr_t;
cs_pimgr_config_t cs_pimgr_default_config(void);
bool cs_pimgr_init(cs_pimgr_t *manager, const cs_pimgr_config_t *config, uint32_t now_ms, uint16_t cmd_seq);
cs_pimgr_output_t cs_pimgr_poll(cs_pimgr_t *manager, const cs_pimgr_input_t *input);
#endif
