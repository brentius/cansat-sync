/* GPL-3.0-only. Flight-software-interfaces section 4 application scheduler.
 * Single main-loop owner; board_init precedes app_init. No allocation. */
#ifndef APP_H
#define APP_H
#include "board.h"
#include "cs_imu.h"
#include "cs_txqueue.h"
#include "cs_flight_state.h"
#include "cs_downlink.h"
#include "cs_pimgr.h"
#include "cs_log.h"
#include "cs_ubx.h"
#include "cs_ntc.h"
#include "cs_telemetry.h"
/* Private standard-runtime declarations also permit bare freestanding builds. */
#if __STDC_HOSTED__
#include <math.h>
#include <string.h>
#else
extern void *memset(void *,int,size_t);
extern void *memcpy(void *,const void *,size_t);
extern double sqrt(double);
#define isfinite(x) __builtin_isfinite(x)
#define NAN __builtin_nan("")
#endif
#include "config.h"
#define APP_LOG_TEMP 0x30u
#define APP_LOG_EVENT 0x31u
#define APP_LOG_RADIO 0x32u
#define APP_LOG_ROUNDTRIP 0x33u
#define APP_PERSIST_BYTES 160u
#define APP_LOG_QUEUE_CAP 44u /* 3520 B + core's two 256 B pages <4 KiB */
#define APP_LOG_PAYLOAD_MAX 64u
typedef struct { uint64_t tick; uint8_t type,len,payload[APP_LOG_PAYLOAD_MAX]; } app_log_record_t;
typedef struct {
    cs_imu_t imu;
    cs_fsm_t fsm;
    cs_downlink_t downlink;
    cs_pimgr_t pi;
    cs_log_t log;
    cs_ubx_t ubx;
    cs_link_decoder_t rx;
    cs_link_encoder_t tx;
    cs_txqueue_t txqueue;
    cs_link_nav_t nav;
    cs_link_gnss_t gnss;
    cs_link_status_t status;
    cs_telemetry_t packet;
    cs_ntc_config_t ntc;
    uint64_t now_us, nav_rx_us, gnss_rx_us, pressure_us, temp_us, accel_us;
    uint64_t last_baro_us, last_status_us, last_persist_us, last_flush_us;
    uint64_t last_fsm_us, temp_on_us, temp_start_us, low_battery_us;
    uint64_t last_log_us[256];
    app_log_record_t log_queue[APP_LOG_QUEUE_CAP];
    size_t log_head,log_count;
    double mission_offset, state_time, pressure_pa, accel_magnitude;
    uint64_t baro_pressure_sum, baro_tick_sum;
    uint32_t baro_tick_base, d2, last_gyro, last_accel, snapshot_tick;
    uint32_t last_pps, previous_pps, last_itow, gnss_config_tick, gnss_baud;
    uint32_t trigger_period, frame, fsm_coalesced, log_suppressed, log_unavailable, saved_protected;
    uint32_t trigger_requested_period; /* Last successful request; actual timer values may round. */
    uint32_t saved_writer,saved_sequence;
    uint32_t radio_failed, gnss_config_attempts, temp_faults, trigger_rejected, log_queue_dropped;
    uint32_t tx_suppressed;
    int32_t origin_lat, origin_lon, origin_h;
    int32_t baro_temperature;
    uint16_t prom[8], telemetry_seq, pps_count;
    uint16_t baro_count, trigger_pulse;
    uint16_t trigger_requested_pulse;
    int16_t temperature_c;
    uint8_t gyro_mod, gnss_config_phase;
    bool initialized, restored, persist_invalid, origin_valid, nav_valid, gnss_valid;
    bool prom_valid, pressure_valid, temp_valid, accel_valid, have_pps, have_itow;
    bool gyro_seen, accel_seen, snapshot_pending, launch_pending;
    bool rx_valid, ack_valid, uart_enabled, pi_alive, temp_disabled, gnss_ack;
    bool log_ready, log_started, log_ring_allowed, shutdown_requested, event_pending;
    bool saved_log_valid,log_resuming,resume_failed;
    bool first_imu_wire,trigger_first,nav_epoch_ready;
    uint8_t pending_event[8];
    cs_link_ack_t ack;
} app_t;
bool app_init(app_t *app);
void app_poll(app_t *app); /* poll board, drain queues, tasks, kick watchdog */
void app_run_once(app_t *app); /* app_poll + board_idle */
void app_build_status(app_t *app, cs_link_status_t *out);
void app_build_packet(app_t *app, cs_telemetry_t *out);
void app_persist_save(app_t *app);
bool app_persist_restore(app_t *app);
double app_mission_time(const app_t *app);
#endif
