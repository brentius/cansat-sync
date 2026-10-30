/* GPL-3.0-only. Board interface: everything the flight application needs
 * from the hardware, and nothing else.
 *
 * Implemented twice:
 *   board/stm32/   the STM32F405 MAIN board (registers, interrupts, DMA)
 *   board/sitl/    host software-in-the-loop, driven by a simulated flight
 * The application (app/) and the core library (core/) use only this header,
 * so the same application code runs in both.
 *
 * Rules every implementation follows:
 *  - Functions are called from the main loop only, never from an interrupt.
 *  - No call blocks for more than about 200 us. Slow device operations
 *    (baro conversions, ADC conversions, flash erase/program, radio TX) are
 *    started by one call and finish in the background or in board_poll().
 *  - Times are TIM2 ticks: 1 MHz, 32 bits, free running from board_init(),
 *    never reset. Differences are taken modulo 2^32.
 *  - Event queues (IMU samples, captures, baro/ADC results) are filled by
 *    interrupts or board_poll() and drained by the *_pop() functions. When
 *    a queue is full the new event is dropped and counted in board_stats().
 *  - board_init() leaves the Pi unpowered, the Pi UART buffer disabled, the
 *    trigger pin low, the radio in standby and excitation off. It starts TIM2
 *    and IMU sampling. It does not wait for GNSS, the Pi or the radio.
 *
 * Owner: main agent (shared interface). Changes are agreed, not made locally.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BOARD_TICK_HZ 1000000u

/* ------------------------------------------------------------------ life */

/* Hello.reset_cause coding (wiki/stm32_pi_link_protocol.md section 5). */
#define BOARD_RESET_POWER    0x01u
#define BOARD_RESET_PIN      0x02u
#define BOARD_RESET_WATCHDOG 0x04u
#define BOARD_RESET_SOFTWARE 0x08u
#define BOARD_RESET_BROWNOUT 0x10u

void board_init(void);
uint8_t board_reset_cause(void);     /* BOARD_RESET_* bits latched at boot */
void board_poll(void);               /* service driver state machines; call every main-loop pass */
void board_watchdog_kick(void);      /* independent watchdog, about 1 s timeout */
void board_idle(void);               /* wait for the next interrupt (WFI); may return at once */
void board_reboot(void);             /* software reset; does not return on target */

/* ------------------------------------------------------------------ time */

uint32_t board_tick(void);           /* TIM2 now */
uint64_t board_time_us(void);        /* TIM2 extended by its wrap count: monotonic since boot */
uint16_t board_tick_hi(void);        /* TIM2 wraps since boot (Hello/Status tick_hi) */

/* ------------------------------------- state kept across resets, not power loss */

/* Boot counter in an RTC backup register: board_init() increments it, so it
 * differs after every reset (Hello/Imu/Status epoch). After power loss it
 * restarts from an arbitrary value. */
uint8_t board_epoch(void);
/* Battery-backed SRAM retained across resets while powered. The application
 * owns the layout and protects it with its own magic and CRC; contents after
 * a power-on reset are undefined. At least 1024 bytes. */
size_t board_persist_size(void);
void board_persist_read(void *dst, size_t offset, size_t n);
void board_persist_write(const void *src, size_t offset, size_t n);

/* ------------------------------------------------------------- IMU BMI088 */

/* One sample in the BMI088's own sensor axes, SI units; the application
 * applies the board-to-body axis mapping. tick is the TIM2 capture of the
 * data-ready edge (gyro INT3 on TIM2_CH2, accel INT1 on TIM2_CH3). */
#define BOARD_IMU_SATURATED 0x01u    /* some axis at full scale */
typedef struct {
    uint32_t tick;
    float v[3];                      /* rad/s (gyro) or m/s^2 (accel) */
    uint8_t flags;                   /* BOARD_IMU_* */
} board_imu_sample_t;

typedef struct {
    bool gyro_ok, accel_ok;          /* chip id read and configured */
    float gyro_odr_hz, accel_odr_hz; /* configured output data rates */
    float gyro_range, accel_range;   /* full scale, rad/s and m/s^2 */
} board_imu_info_t;

void board_imu_info(board_imu_info_t *info);
bool board_gyro_pop(board_imu_sample_t *s);
bool board_accel_pop(board_imu_sample_t *s);

/* ------------------------------------------------------- barometer MS5611 */

/* The board runs OSR-4096 conversions continuously: pressure (D1) every
 * time and temperature (D2) every board-chosen Nth conversion. Each result
 * is queued raw; compensation is the core library's job. tick is the middle
 * of the conversion. */
typedef struct {
    uint32_t tick;
    uint32_t adc;                    /* 24-bit raw result */
    bool is_temperature;             /* D2 if true, D1 if false */
} board_baro_raw_t;

bool board_baro_prom(uint16_t prom[8]);  /* false until PROM was read; CRC is checked by the caller */
bool board_baro_ok(void);                /* responding */
bool board_baro_pop(board_baro_raw_t *r);

/* --------------------------------------------- air temperature ADS1115 + NTC */

/* One reading = excitation on, settle, AIN0 (TEMP_BUF, divider) then AIN1
 * (TEMP_EXC, excitation) single-ended single-shot conversions, excitation
 * off. The board reports raw codes, their PGA full scales and the measured
 * excitation on-time so the caller can convert and account duty. */
typedef struct {
    uint32_t tick;                   /* middle of the AIN0 conversion */
    int16_t code_ain0, code_ain1;
    uint16_t fs_mv_ain0, fs_mv_ain1; /* PGA full scale for each, e.g. 2048, 4096 */
    uint32_t excitation_us;          /* how long excitation was on for this reading */
} board_temp_raw_t;

typedef struct {
    uint32_t period_us;              /* time between readings, e.g. 100000 */
    uint32_t settle_us;              /* excitation on before the first conversion starts */
    uint16_t sps;                    /* ADS1115 data rate: 8..860 */
    uint16_t fs_mv_ain0, fs_mv_ain1; /* requested PGA full scales */
} board_temp_cfg_t;

bool board_temp_configure(const board_temp_cfg_t *cfg);  /* false if not supported; readings stop if period_us == 0 */
bool board_temp_ok(void);
bool board_temp_pop(board_temp_raw_t *r);

/* ---------------------------------------------- GNSS MAX-M10S on USART3 */

size_t board_gnss_read(uint8_t *buf, size_t max);       /* bytes received since the last call */
bool board_gnss_write(const uint8_t *buf, size_t n);    /* queue for transmit; false if the TX buffer is busy */
bool board_gnss_tx_idle(void);
void board_gnss_set_baud(uint32_t baud);                /* after the module was told to change */
void board_gnss_reset(bool asserted);                   /* GNSS_RESET_N (open drain) */
bool board_pps_pop(uint32_t *tick);                     /* TIM2_CH1 rising-edge captures */

/* --------------------------------------------- Raspberry Pi power and UART */

void board_pi_power(bool on);        /* PI_PWR_EN */
void board_pi_uart_enable(bool on);  /* drives PI_UART_OE_N low when on */
bool board_pi_powered(void);         /* PI_PWR_EN state as driven */
bool board_pgood_5v(void);
bool board_pgood_3v3(void);
size_t board_pi_read(uint8_t *buf, size_t max);         /* bytes received since the last call */
/* Start sending n bytes (copied; n <= BOARD_PI_TX_MAX). False while the
 * previous transmission is still in progress. */
#define BOARD_PI_TX_MAX 512u
bool board_pi_write(const uint8_t *buf, size_t n);
bool board_pi_tx_idle(void);

/* --------------------------------------------- camera trigger (TIM3_CH3 PB0) */

/* Start periodic low pulses on PB0 (idle high while running). Returns false
 * and leaves triggers stopped if the timer cannot produce them. The values
 * actually produced are returned through the pointers (rounded to the
 * timer's resolution) and are what Trig messages must report. */
bool board_trig_start(uint32_t period_us, uint16_t pulse_us, uint32_t *actual_period_us,
                      uint16_t *actual_pulse_us);
/* Stop triggers. drive_low: hold PB0 low (camera unpowered or Pi shutting
 * down) instead of idle high. */
void board_trig_stop(bool drive_low);
bool board_trig_running(void);
bool board_trig_pop(uint32_t *tick);                    /* TIM2_CH4 falling-edge captures */

/* ------------------------------------------------ LoRa radio E22-900M22S */

typedef struct {
    uint32_t frequency_hz;           /* 869525000 */
    uint8_t sf;                      /* 7 */
    uint32_t bw_hz;                  /* 125000 */
    uint8_t cr;                      /* 1 = 4/5 ... 4 = 4/8 */
    uint16_t preamble;               /* 8 */
    bool crc_on, implicit_header, ldro;
    int8_t power_dbm;                /* SX1262 output setting */
    uint16_t tcxo_mv;                /* DIO3 TCXO supply, 0 for none */
} board_radio_cfg_t;

bool board_radio_configure(const board_radio_cfg_t *cfg);  /* blocking up to a few ms, call at start-up */
bool board_radio_ok(void);
/* Start transmitting n <= 255 bytes. False if busy or faulted. TXEN/RXEN
 * are handled by the board; the radio returns to standby when done. */
bool board_radio_send(const uint8_t *buf, size_t n);
bool board_radio_busy(void);
uint32_t board_radio_tx_count(void); /* completed transmissions */

/* ------------------------------------------------ NOR flash W25Q128 */

#define BOARD_FLASH_PAGE   256u
#define BOARD_FLASH_SECTOR 4096u
bool board_flash_ok(void);           /* JEDEC id matched at start-up */
uint32_t board_flash_size(void);     /* bytes */
bool board_flash_busy(void);         /* erase or program in progress */
bool board_flash_read(uint32_t addr, uint8_t *buf, size_t n);          /* n <= 256; false if busy */
bool board_flash_program(uint32_t addr, const uint8_t *buf, size_t n); /* starts; within one page */
bool board_flash_erase_sector(uint32_t addr);                          /* starts; 4 KiB aligned */

/* ------------------------------------------------ housekeeping and recovery */

uint16_t board_battery_mv(void);     /* pack voltage from the PC0 divider, 0 if unavailable */
int8_t board_mcu_temp_c(void);
#define BOARD_LED_STATUS 0
#define BOARD_LED_FAULT  1
void board_led(int led, bool on);
void board_buzzer(bool on);          /* about 4 kHz on TIM4_CH3 */

typedef struct {
    uint32_t gyro_dropped, accel_dropped, baro_dropped, temp_dropped;
    uint32_t pps_dropped, trig_dropped;
    uint32_t pi_rx_overrun, gnss_rx_overrun;
    uint32_t i2c_errors, spi_errors, radio_errors, flash_errors;
} board_stats_t;
const board_stats_t *board_stats(void);

#endif
