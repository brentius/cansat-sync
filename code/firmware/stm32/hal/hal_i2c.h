/* GPL-3.0-only. RM0090 I2C master IRQ state machine and GPIO bus recovery.
 * Implements drivers/drv_bus.h for one sensor per bus; sources in SOURCES.md. */
#ifndef HAL_I2C_H
#define HAL_I2C_H
#include "stm32f405.h"
#include "../drivers/drv_bus.h"
typedef struct {
    hal_i2c_regs_t *regs;
    uint32_t hz, deadline, recovery_tick;
    uint32_t *errors;
    uint8_t tx[4],rx[16],addr,nt,nr,pos,scl,sda;
    volatile unsigned state;
    volatile int completion;
    unsigned recovery;
} hal_i2c_t;
void hal_i2c_init(hal_i2c_t *,hal_i2c_regs_t *,unsigned scl,unsigned sda,uint32_t hz,uint32_t *errors);
bool hal_i2c_start(void *,uint8_t,const uint8_t *,size_t,size_t);
int hal_i2c_result(void *,uint8_t *,size_t);
void hal_i2c_irq(hal_i2c_t *);
void hal_i2c_poll(hal_i2c_t *,uint32_t);
#endif
