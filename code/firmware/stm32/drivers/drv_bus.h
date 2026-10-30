/* GPL-3.0-only. Bounded transport seam for the flight-software HAL package.
 * Implementations and test devices must copy TX before returning from start.
 * I2C result consumes a completed transaction: 0 pending, 1 done, -1 failed.
 * One owner per I2C bus. SPI is one complete CS assertion, mode 0, MSB first. */
#ifndef DRV_BUS_H
#define DRV_BUS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct {
    void *ctx;
    bool (*start)(void *, uint8_t, const uint8_t *, size_t, size_t);
    int (*result)(void *, uint8_t *, size_t);
} drv_i2c_t;
typedef enum { DRV_ACCEL, DRV_GYRO, DRV_FLASH, DRV_RADIO } drv_spi_device_t;
typedef struct {
    void *ctx;
    bool (*transfer)(void *, drv_spi_device_t, const uint8_t *, uint8_t *, size_t);
} drv_spi_t;
static inline bool drv_due(uint32_t now, uint32_t deadline)
{ return (int32_t)(now-deadline)>=0; }
#endif
