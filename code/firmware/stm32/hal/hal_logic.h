/* GPL-3.0-only. Host-testable queues/timer/DMA arithmetic for board.h. */
#ifndef HAL_LOGIC_H
#define HAL_LOGIC_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct {
    uint8_t *data;
    size_t item_size;
    uint32_t capacity;
    volatile uint32_t head, tail;
    uint32_t write_index, read_index;
} hal_ring_t;
void hal_ring_init(hal_ring_t *,void *,size_t,uint32_t);
/* SPSC; caller masks interrupts around pop on target, push publishes last. */
bool hal_ring_push(hal_ring_t *,const void *);
bool hal_ring_pop(hal_ring_t *,void *);
/* Caller excludes producer (IRQ masked/stopped). Returns discarded events. */
uint32_t hal_ring_discard(hal_ring_t *);
size_t hal_dma_available(uint32_t produced,uint32_t *consumed,uint32_t capacity,uint32_t *lost);
/* 8N1 wire time plus 20 ms transport margin; n is bounded by UART TX cap. */
uint32_t hal_uart_transfer_budget(uint32_t n,uint32_t baud);
bool hal_trig_values(uint32_t period,uint16_t pulse,uint16_t *psc,uint16_t *arr,
                     uint16_t *ccr,uint32_t *actual_period,uint16_t *actual_pulse);
/* DS8626 factory ADC values; zero/erased calibration falls back to typical
 * VREFINT/V25/slope. This is housekeeping, not air-temperature sensing. */
void hal_adc_convert(uint16_t battery,uint16_t temperature,uint16_t vref,
                     uint16_t cal_vref,uint16_t cal30,uint16_t cal110,
                     uint16_t *battery_mv,int8_t *temperature_c);
#endif
