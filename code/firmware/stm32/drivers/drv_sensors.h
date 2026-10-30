/* GPL-3.0-only. BMI088, MS5611 and ADS1115 acquisition, implementing board.h.
 * Register provenance, timing bounds and candidate choices: SOURCES.md.
 * No compensation here: core/cs_ms5611 and core/cs_ntc own that policy. */
#ifndef DRV_SENSORS_H
#define DRV_SENSORS_H
#include "drv_bus.h"
#include "../board/board.h"
typedef struct {
    drv_spi_t bus;
    uint32_t deadline, errors;
    unsigned step;
    bool gyro_ok, accel_ok;
} drv_bmi_t;
void drv_bmi_init(drv_bmi_t *, drv_spi_t, uint32_t);
void drv_bmi_poll(drv_bmi_t *, uint32_t);
/* Call only after the captured DRDY. Exactly one six-byte sample read. */
bool drv_bmi_raw(drv_bmi_t *, bool accel, uint8_t raw[6]);
typedef struct { uint32_t tick; uint8_t raw[6]; } drv_bmi_raw_sample_t;
bool drv_bmi_capture(drv_bmi_t *,bool accel,uint32_t tick,bool overcapture,
                     drv_bmi_raw_sample_t *,uint32_t *dropped);
void drv_bmi_decode(bool accel, const uint8_t raw[6], uint32_t, board_imu_sample_t *);
void drv_bmi_info(const drv_bmi_t *, board_imu_info_t *);
typedef struct {
    drv_i2c_t bus;
    uint32_t deadline, conversion_tick, errors;
    uint16_t prom[8];
    unsigned state, index, pressure_count;
    bool ready, ok, temperature;
} drv_baro_t;
void drv_baro_init(drv_baro_t *, drv_i2c_t, uint32_t);
/* true emits a conversion; caller owns bounded event queue/drop counting. */
bool drv_baro_poll(drv_baro_t *, uint32_t, board_baro_raw_t *);
typedef struct {
    void *ctx;
    /* Hardware MUST force off after budget_us even if poll stops. Off returns
     * the actual elapsed on-time, including an earlier hardware cutoff. */
    uint32_t (*excite)(void *, bool on, uint32_t budget_us);
    bool (*is_on)(void *);
} drv_excitation_t;
typedef struct {
    drv_i2c_t bus;
    drv_excitation_t gate;
    board_temp_cfg_t cfg;
    uint32_t next, deadline, cutoff, conv_tick, budget, errors;
    unsigned state, channel;
    int16_t codes[2];
    bool ok;
} drv_temp_t;
bool drv_ads_config_word(uint16_t fs_mv, uint16_t sps, unsigned channel, uint16_t *);
void drv_temp_init(drv_temp_t *, drv_i2c_t, drv_excitation_t);
bool drv_temp_configure(drv_temp_t *, const board_temp_cfg_t *, uint32_t);
bool drv_temp_poll(drv_temp_t *, uint32_t, board_temp_raw_t *);
#endif
