/* GPL-3.0-only. C twin of link.crc / telemetry.crc16. */
#ifndef CS_CRC16_H
#define CS_CRC16_H
#include <stddef.h>
#include <stdint.h>
uint16_t cs_crc16(const uint8_t *data, size_t len, uint16_t initial);
#endif
