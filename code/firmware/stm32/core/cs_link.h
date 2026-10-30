/* GPL-3.0-only. C twin of cansat/link.py; structs contain wire-coded integers,
 * including fields whose Python names end in _pa, _m, _rad, _s or _v.
 * Pack/unpack use explicit little endian; reserved padding is zero on pack.
 * Length-returning functions return 0 on capacity/argument failure. */
#ifndef CS_LINK_H
#define CS_LINK_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define CS_LINK_MAX_PAYLOAD 249u
#define CS_LINK_MAX_FRAME 253u
#define CS_LINK_MAX_WIRE 255u
#define CS_FLAG_CAMERA_OK 128u
#define CS_FLAG_PRED_VALID 64u
#define CS_FLAG_TRN_VALID 32u
#define CS_PROTO_VERSION 1u
#define CS_TICK_HZ 1000000u
#define CS_TICK_WRAP 4294967296u
#define CS_MAX_PAYLOAD 249u
#define CS_MAX_FRAME 253u
#define CS_MAX_WIRE 255u
#define CS_OVERHEAD 6u
#define CS_Q_ONE 1073741824u
#define CS_RESET_POWER 1u
#define CS_RESET_PIN 2u
#define CS_RESET_WATCHDOG 4u
#define CS_RESET_SOFTWARE 8u
#define CS_RESET_BROWNOUT 16u
#define CS_IMU_GYRO_SATURATED 1u
#define CS_IMU_ACCEL_SATURATED 2u
#define CS_IMU_GYRO_MISSED 4u
#define CS_IMU_ACCEL_MISSED 8u
#define CS_IMU_RESTART 16u
#define CS_IMU_GYRO_FAULT 32u
#define CS_IMU_ACCEL_FAULT 64u
#define CS_GNSS_TICK_FROM_PPS 1u
#define CS_GNSS_STALE 2u
#define CS_HEALTH_5V 1u
#define CS_HEALTH_3V3 2u
#define CS_HEALTH_PI_POWER 4u
#define CS_HEALTH_UART 8u
#define CS_HEALTH_IMU 16u
#define CS_HEALTH_BARO 32u
#define CS_HEALTH_GNSS 64u
#define CS_HEALTH_FIX 128u
#define CS_HEALTH_PPS 256u
#define CS_HEALTH_RADIO 512u
#define CS_HEALTH_FLASH 1024u
#define CS_HEALTH_TEMP 2048u
#define CS_HEALTH_TRIGGERS 4096u
#define CS_HEALTH_ORIGIN 8192u
#define CS_HEALTH_BASELINE 16384u
#define CS_HEALTH_PI_ALIVE 32768u
#define CS_CMD_SHUTDOWN 1u
#define CS_CMD_PREPARE 2u
#define CS_ACK_ACCEPTED 0u
#define CS_ACK_DONE 1u
#define CS_ACK_REFUSED 2u
#define CS_ACK_UNKNOWN 3u
#define CS_PI_BOOTING 0u
#define CS_PI_READY 1u
#define CS_PI_CAMERA 2u
#define CS_PI_NAVIGATING 3u
#define CS_PI_STOPPING 4u
#define CS_PI_FAULT 5u
#define CS_PIFLAG_CAMERA 1u
#define CS_PIFLAG_MAP 2u
#define CS_PIFLAG_STORAGE 4u
#define CS_PIFLAG_UNDERVOLT 8u
#define CS_PIFLAG_THROTTLED 16u
#define CS_PIFLAG_NAV 32u
#define CS_HELLO_REQUIRE_EPOCH_ECHO 1u
#define CS_PIFLAG_EPOCH_ECHO 0x0040u
#define CS_PIFLAG_EPOCH_SHIFT 8u
#define CS_PIFLAG_EPOCH_MASK 0xff00u
#define CS_LINK_HELLO_ID 0x01u
#define CS_LINK_HELLO_SIZE 16u
typedef struct {
    uint8_t proto;
    uint8_t epoch;
    uint16_t fw_version;
    uint8_t reset_cause;
    uint8_t flags;
    uint16_t tick_hi;
    uint32_t tick;
    uint32_t tick_hz;
} cs_link_hello_t;
size_t cs_link_pack_hello(uint8_t *dst, size_t cap, const cs_link_hello_t *msg);
bool cs_link_unpack_hello(cs_link_hello_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_IMU_ID 0x10u
#define CS_LINK_IMU_SIZE 36u
typedef struct {
    uint32_t tick;
    int32_t q[4];
    int32_t u[3];
    uint8_t epoch;
    uint8_t flags;
    uint8_t n_gyro;
    uint8_t n_accel;
} cs_link_imu_t;
size_t cs_link_pack_imu(uint8_t *dst, size_t cap, const cs_link_imu_t *msg);
bool cs_link_unpack_imu(cs_link_imu_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_BARO_ID 0x11u
#define CS_LINK_BARO_SIZE 12u
typedef struct {
    uint32_t tick;
    uint32_t pressure_pa;
    int16_t temperature_c;
    uint8_t n_samples;
    uint8_t flags;
} cs_link_baro_t;
size_t cs_link_pack_baro(uint8_t *dst, size_t cap, const cs_link_baro_t *msg);
bool cs_link_unpack_baro(cs_link_baro_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_GNSS_ID 0x12u
#define CS_LINK_GNSS_SIZE 60u
typedef struct {
    uint32_t tick;
    uint32_t itow_ms;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t valid;
    uint8_t fix_type;
    uint8_t fix_flags;
    uint8_t num_sv;
    uint8_t link_flags;
    int32_t lon_deg;
    int32_t lat_deg;
    int32_t h_msl_m;
    uint32_t h_acc_m;
    uint32_t v_acc_m;
    int32_t vel_n;
    int32_t vel_e;
    int32_t vel_d;
    uint32_t s_acc;
    uint16_t p_dop;
} cs_link_gnss_t;
size_t cs_link_pack_gnss(uint8_t *dst, size_t cap, const cs_link_gnss_t *msg);
bool cs_link_unpack_gnss(cs_link_gnss_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_TRIG_ID 0x13u
#define CS_LINK_TRIG_SIZE 16u
typedef struct {
    uint32_t frame;
    uint32_t tick;
    uint32_t period_us;
    uint16_t pulse_us;
    uint8_t flags;
} cs_link_trig_t;
size_t cs_link_pack_trig(uint8_t *dst, size_t cap, const cs_link_trig_t *msg);
bool cs_link_unpack_trig(cs_link_trig_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_PPS_ID 0x14u
#define CS_LINK_PPS_SIZE 16u
typedef struct {
    uint32_t tick;
    uint32_t itow_ms;
    uint32_t interval;
    uint16_t count;
    uint8_t flags;
} cs_link_pps_t;
size_t cs_link_pack_pps(uint8_t *dst, size_t cap, const cs_link_pps_t *msg);
bool cs_link_unpack_pps(cs_link_pps_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_STATUS_ID 0x15u
#define CS_LINK_STATUS_SIZE 48u
typedef struct {
    uint32_t tick;
    uint16_t tick_hi;
    uint8_t epoch;
    uint8_t state;
    uint32_t tick_state;
    uint16_t health;
    uint16_t battery_v;
    uint32_t baseline_pa;
    int32_t origin_lat_deg;
    int32_t origin_lon_deg;
    int32_t origin_h_msl_m;
    int16_t altitude_m;
    int16_t temperature_c;
    uint16_t telemetry_seq;
    uint16_t rx_crc;
    uint16_t rx_lost;
    uint16_t tx_dropped;
    uint8_t pi_age_s;
    int8_t mcu_temp_c;
} cs_link_status_t;
size_t cs_link_pack_status(uint8_t *dst, size_t cap, const cs_link_status_t *msg);
bool cs_link_unpack_status(cs_link_status_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_CMD_ID 0x16u
#define CS_LINK_CMD_SIZE 8u
typedef struct {
    uint32_t tick;
    uint8_t cmd_seq;
    uint8_t cmd;
    uint16_t arg;
} cs_link_cmd_t;
size_t cs_link_pack_cmd(uint8_t *dst, size_t cap, const cs_link_cmd_t *msg);
bool cs_link_unpack_cmd(cs_link_cmd_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_PI_STATUS_ID 0x80u
#define CS_LINK_PI_STATUS_SIZE 24u
typedef struct {
    uint32_t uptime_ms;
    uint32_t tick_echo;
    uint16_t echo_age_ms;
    uint8_t proto;
    uint8_t state;
    uint16_t flags;
    uint16_t sw_version;
    uint16_t rx_crc;
    uint16_t rx_lost;
    uint16_t frames;
    int8_t cpu_temp_c;
    uint8_t cpu_load;
} cs_link_pi_status_t;
size_t cs_link_pack_pi_status(uint8_t *dst, size_t cap, const cs_link_pi_status_t *msg);
bool cs_link_unpack_pi_status(cs_link_pi_status_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_NAV_ID 0x81u
#define CS_LINK_NAV_SIZE 32u
typedef struct {
    uint32_t tick;
    uint16_t frame;
    uint8_t flags;
    uint8_t landmarks;
    int16_t trn_e;
    int16_t trn_n;
    int16_t trn_u;
    uint8_t trn_major_m;
    uint8_t trn_minor_m;
    uint8_t trn_angle_rad;
    uint8_t protection_m;
    int16_t pred_e;
    int16_t pred_n;
    uint8_t time_to_go_s;
    uint8_t pred_major_m;
    uint8_t pred_minor_m;
    uint8_t pred_angle_rad;
    int16_t vel_e;
    int16_t vel_n;
    int16_t vel_u;
} cs_link_nav_t;
size_t cs_link_pack_nav(uint8_t *dst, size_t cap, const cs_link_nav_t *msg);
bool cs_link_unpack_nav(cs_link_nav_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_ACK_ID 0x82u
#define CS_LINK_ACK_SIZE 4u
typedef struct {
    uint8_t cmd_seq;
    uint8_t cmd;
    uint8_t result;
} cs_link_ack_t;
size_t cs_link_pack_ack(uint8_t *dst, size_t cap, const cs_link_ack_t *msg);
bool cs_link_unpack_ack(cs_link_ack_t *msg, const uint8_t *src, size_t len);
#define CS_LINK_TRIG_CFG_ID 0x83u
#define CS_LINK_TRIG_CFG_SIZE 8u
typedef struct {
    uint8_t enable;
    uint8_t cfg_seq;
    uint16_t period_ms;
    uint16_t pulse_us;
} cs_link_trig_cfg_t;
size_t cs_link_pack_trig_cfg(uint8_t *dst, size_t cap, const cs_link_trig_cfg_t *msg);
bool cs_link_unpack_trig_cfg(cs_link_trig_cfg_t *msg, const uint8_t *src, size_t len);
/* Scaling: link floor(x/step+0.5), reserve signed minimum/unsigned maximum for NA. */
int32_t cs_link_scale_signed(double value, double step, unsigned bits);
uint32_t cs_link_scale_unsigned(double value, double step, unsigned bits);
uint8_t cs_link_angle(double radians);
size_t cs_link_payload_size(uint8_t id);
size_t cs_link_frame(uint8_t *dst, size_t cap, uint8_t id, uint8_t seq, const uint8_t *payload, size_t len);
typedef struct { uint8_t seq; } cs_link_encoder_t;
void cs_link_encoder_init(cs_link_encoder_t *encoder);
size_t cs_link_encode(cs_link_encoder_t *encoder, uint8_t *dst, size_t cap, uint8_t id, const uint8_t *payload, size_t len);
typedef struct {
    uint64_t bytes, frames, empty, too_long, stuffing, short_frame, crc, unknown, length, lost, repeats, ignored, faults;
} cs_link_counters_t;
/* short_frame corresponds to Python stats["short"]. Callback payload valid only during callback. */
typedef void (*cs_link_receive_fn)(void *context, uint8_t id, uint8_t seq, const uint8_t *payload, size_t len);
typedef struct {
    uint8_t buf[CS_LINK_MAX_WIRE - 1u], last_seq;
    size_t used;
    bool skip, have_seq, raw_unknown;
    cs_link_counters_t stats;
} cs_link_decoder_t;
void cs_link_decoder_init(cs_link_decoder_t *decoder, bool raw_unknown);
void cs_link_decoder_reset_sequence(cs_link_decoder_t *decoder);
void cs_link_decoder_feed(cs_link_decoder_t *decoder, const uint8_t *data, size_t len, cs_link_receive_fn receive, void *context);
#endif
