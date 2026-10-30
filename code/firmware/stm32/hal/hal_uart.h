/* GPL-3.0-only. USART1/3 DMA transport for board.h Pi and GNSS byte APIs. */
#ifndef HAL_UART_H
#define HAL_UART_H
#include "stm32f405.h"
#include <stddef.h>
#define HAL_UART_RX_CAP 2048u
#define HAL_UART_TX_CAP 512u
typedef struct {
    hal_usart_t *regs;
    hal_dma_stream_t *rx,*tx;
    uint32_t dma,rx_stream,tx_stream,pclk,baud,consumed,deadline;
    volatile uint32_t wraps;
    uint32_t *errors;
    bool transmitting;
    volatile uint8_t rx_data[HAL_UART_RX_CAP];
    uint8_t tx_data[HAL_UART_TX_CAP];
} hal_uart_t;
void hal_uart_init(hal_uart_t *,bool pi,uint32_t *errors);
void hal_uart_poll(hal_uart_t *,uint32_t);
void hal_uart_irq(hal_uart_t *);
void hal_uart_rx_irq(hal_uart_t *);
void hal_uart_baud(hal_uart_t *,uint32_t);
size_t hal_uart_read(hal_uart_t *,uint8_t *,size_t);
bool hal_uart_write(hal_uart_t *,const uint8_t *,size_t,uint32_t);
bool hal_uart_idle(hal_uart_t *);
void hal_uart_discard(hal_uart_t *,bool abort_tx);
#endif
