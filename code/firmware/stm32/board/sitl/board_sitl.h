/* GPL-3.0-only. Host-only controls for the board.h SITL twin.
 * Synthetic sensors, timed UART/radio/NOR. No physical measurement. */
#ifndef BOARD_SITL_H
#define BOARD_SITL_H
#include "board.h"
#define BOARD_SITL_FLASH_MAX (16u*1024u*1024u)
#define BOARD_SITL_LINK 0
#define BOARD_SITL_RADIO 1
#define BOARD_SITL_GNSS_TX 2
typedef void (*board_sitl_sink_t)(void *ctx,int channel,const uint8_t *data,size_t n);
typedef struct {
    bool gyro,accel,baro,temp,gnss,radio,flash,pgood5,pgood3;
    bool gyro_saturated,accel_saturated,temp_open;
    float gyro_v[3],accel_v[3];
    double pressure_pa;
    uint32_t temp_excitation_us;
    uint16_t battery_mv;
} board_sitl_inputs_t;
typedef struct {
    uint64_t watchdog_kicks,watchdog_max_gap_us,flash_reads,flash_programs,flash_erases;
    uint32_t flash_rule_errors,power_order_errors,trigger_starts,gnss_writes;
    uint32_t max_reads_per_poll,max_flash_ops_per_poll;
    uint32_t baud; bool uart_enabled,trigger_low,buzzer,leds[2];
} board_sitl_stats_t;
void board_sitl_cold_start(uint32_t flash_size);
void board_sitl_warm_reset(uint8_t cause); /* board_init required afterwards */
void board_sitl_advance(uint32_t us);
void board_sitl_set_time(uint64_t us); /* test capture wrap; use before app_init */
board_sitl_inputs_t *board_sitl_inputs(void);
const board_sitl_stats_t *board_sitl_stats(void);
void board_sitl_set_sink(board_sitl_sink_t sink,void *ctx);
bool board_sitl_pi_receive(const uint8_t *p,size_t n);
bool board_sitl_gnss_receive(const uint8_t *p,size_t n);
bool board_sitl_pps(uint32_t tick);
const uint8_t *board_sitl_flash_data(void);
void board_sitl_corrupt_persist(size_t offset,uint8_t value);
#endif
