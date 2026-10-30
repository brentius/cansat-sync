/* GPL-3.0-only. W25Q128JV and SX1262 board.h backends; see SOURCES.md.
 * Poll never waits for BUSY or nonvolatile writes. Flash startup probing may
 * issue four short commands once BUSY clears; ordinary polling uses one. */
#ifndef DRV_DEVICES_H
#define DRV_DEVICES_H
#include "drv_bus.h"
#include "../board/board.h"
typedef struct {
    drv_spi_t bus;
    uint32_t deadline, next_poll, errors;
    bool ok, busy, probing;
} drv_flash_t;
void drv_flash_init(drv_flash_t *, drv_spi_t);
void drv_flash_poll(drv_flash_t *, uint32_t);
bool drv_flash_read(drv_flash_t *,uint32_t,uint8_t *,size_t);
bool drv_flash_program(drv_flash_t *,uint32_t,const uint8_t *,size_t,uint32_t);
bool drv_flash_erase(drv_flash_t *,uint32_t,uint32_t);
typedef struct {
    void *ctx;
    bool (*busy)(void *);
    void (*reset)(void *,bool asserted);
    void (*txen)(void *,bool on);
} drv_radio_io_t;
typedef struct {
    drv_spi_t bus;
    drv_radio_io_t io;
    board_radio_cfg_t cfg;
    uint32_t deadline, next_poll, errors, tx_count;
    uint8_t packet[255], len, offset, stage, tx_stage;
    bool configured, ok, transmitting;
} drv_radio_t;
void drv_radio_init(drv_radio_t *,drv_spi_t,drv_radio_io_t,uint32_t);
bool drv_radio_configure(drv_radio_t *,const board_radio_cfg_t *,uint32_t);
bool drv_radio_send(drv_radio_t *,const uint8_t *,size_t,uint32_t);
void drv_radio_poll(drv_radio_t *,uint32_t);
uint32_t drv_radio_pll(uint32_t hz);
#endif
