/* GPL-3.0-only. CRC-16/CCITT-FALSE, twin of link.crc. */
#include "cs_crc16.h"

uint16_t cs_crc16(const uint8_t *data, size_t len, uint16_t initial)
{
    size_t i;
    uint16_t crc = initial;
    for (i = 0; i < len; i++) {
        unsigned bit;
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; bit++)
            crc = (uint16_t)((crc & 0x8000u) ? ((uint32_t)crc << 1) ^ 0x1021u : (uint32_t)crc << 1);
    }
    return crc;
}
