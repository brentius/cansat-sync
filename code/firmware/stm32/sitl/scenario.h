/* GPL-3.0-only. Synthetic flight stimuli for app host tests and replay. */
#ifndef APP_SCENARIO_H
#define APP_SCENARIO_H
#include "app.h"
#include "board_sitl.h"
typedef struct {
    cs_link_encoder_t encoder;
    uint64_t next_pi,next_nav,next_gnss,next_pps,next_cfg;
    uint32_t receiver_baud;
    bool pi_enabled,gnss_enabled,sensor_failures;
    bool ack_shutdown;
    uint8_t last_shutdown_seq;
    bool shutdown_seen;
    cs_link_decoder_t decoder;
    uint64_t link_bytes,radio_bytes,frames[256];
    uint32_t radio_packets,radio_bad,imu_restart,imu_missed,imu_faults;
    uint32_t state_mask,prepare,shutdown;
    cs_telemetry_t last_packet;
    void (*capture)(void *,int,const uint8_t *,size_t);
    void *capture_ctx;
} app_scenario_t;
void app_scenario_init(app_scenario_t *s);
void app_scenario_sink(void *ctx,int channel,const uint8_t *p,size_t n);
void app_scenario_step(app_scenario_t *s,double world_s);
double app_scenario_altitude(double t);
#endif
