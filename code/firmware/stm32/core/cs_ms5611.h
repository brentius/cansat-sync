/* GPL-3.0-only. MS5611-01BA03 compensation, datasheet pp 7-9 / AN520 CRC.
 * Integer intermediates retain Q8 fractional Pa (instead of rounding to Pa);
 * pressure_cpa is rounded from Q8 to the link's 0.01 Pa. No float arithmetic.
 * PROM valid check is separate; compensate also rejects invalid CRC/raw ADC. */
#ifndef CS_MS5611_H
#define CS_MS5611_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { int32_t temperature_centi_c; uint32_t pressure_cpa; int64_t pressure_q8_pa; } cs_ms5611_result_t;
uint8_t cs_ms5611_crc4(const uint16_t prom[8]);
bool cs_ms5611_prom_valid(const uint16_t prom[8]);
bool cs_ms5611_compensate(const uint16_t prom[8], uint32_t d1, uint32_t d2,
                         bool second_order, cs_ms5611_result_t *out);
#endif
