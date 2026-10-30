/* GPL-3.0-only. Candidate policies for flight-software-interfaces section 4.
 * All values are [assumed] candidate settings, NOT hardware qualification.
 * Core sensor conversion coefficients retain their documented provenance. */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H
#define APP_FW_VERSION 0x0002u
#define APP_BARO_INTERVAL_US 100000u
#define APP_STATUS_INTERVAL_US 1000000u
#define APP_PERSIST_INTERVAL_US 1000000u
#define APP_LOG_FLUSH_US 1000000u
#define APP_LANDED_LOG_US 10000000u
#define APP_PRELAUNCH_RETAIN_US UINT64_C(60000000)
#define APP_SENSOR_STALE_US 500000u
#define APP_TEMP_STALE_US 1000000u
#define APP_GNSS_STALE_US 2000000u
#define APP_ACCEL_STALE_US 50000u
#define APP_IMU_GYRO_GAP_US 1500u
#define APP_IMU_ACCEL_GAP_US 1000u
#define APP_IMU_SNAPSHOT_GYROS 10u
#define APP_TEMP_PERIOD_US 125000u
#define APP_TEMP_SETTLE_US 2300u
#define APP_TEMP_SPS 860u
/* Driver reserves settle + 2*(ceil(1120000/SPS)+25+500)+300 = 6256 us.
 * 6256 / 125000 = 5.0048%, preserving settle margin at 8 Hz. */
#define APP_TEMP_MAX_ON_US 7500u
#define APP_ORIGIN_MAX_HACC_MM 5000u
#define APP_SHUTDOWN_LANDED_S 300.0
/* 2S pack. The radio packet's battery byte (2.50-5.04 V) carries the mean
 * cell voltage; Status keeps the whole-pack millivolts. */
#define APP_BATTERY_CELLS 2u
#define APP_LOW_BATTERY_MV 6500u
#define APP_LOW_BATTERY_HOLD_US 5000000u
#define APP_GNSS_RETRY_US 10000000u
#define APP_GNSS_ACK_US 500000u
#define APP_GNSS_SETTLE_US 500000u
#define APP_TRIG_MIN_PERIOD_US 50000u
#define APP_TRIG_MAX_PERIOD_US 10000000u
#define APP_TRIG_MAX_PULSE_US 20000u
#define APP_RADIO_FREQUENCY_HZ 869525000u
#define APP_RADIO_POWER_DBM 14
/* TCXO 1.8 V is an [assumed] placeholder pending module/manual confirmation. */
#define APP_RADIO_TCXO_MV 1800u
#define APP_RADIO_DUTY 0.10
#define APP_RADIO_WINDOW_S 3600.0
#define APP_RADIO_FLIGHT_INTERVAL_S 1.0
#define APP_RADIO_FLIGHT_RESERVE_S 240.0
/* 1 s on PAD/LANDED as well (mass_power_budget.md): primary 1 Hz telemetry
 * must not depend on launch detection. 8.2% airtime fits the 10% limit with
 * the flight reserve held. */
#define APP_RADIO_GROUND_INTERVAL_S 1.0
/* Identity is an [assumed] board/sensor mounting pending physical axis survey.
 * Separate gyro and accel matrices permit sensors' package orientations. */
static const float app_gyro_to_body[3][3]={{1,0,0},{0,1,0},{0,0,1}};
static const float app_accel_to_body[3][3]={{1,0,0},{0,1,0},{0,0,1}};
#endif
