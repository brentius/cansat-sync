/* GPL-3.0-only. u-blox M10 SPG 5.10 Interface description UBX-21035062 R03:
 * UBX frame/checksum, NAV-PVT and CFG-VALSET. Max payload 256, no allocation.
 * Feed returns one frame valid until next feed. Oversize frames rejected at
 * length header and sync search resumes; checksum/oversize counters saturate
 * only at uint32 wrap. NAV-PVT integers are copied into link wire units. */
#ifndef CS_UBX_H
#define CS_UBX_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cs_link.h"
#define CS_UBX_MAX_PAYLOAD 256u
typedef struct { uint8_t cls, id; uint16_t len; uint8_t payload[CS_UBX_MAX_PAYLOAD]; } cs_ubx_frame_t;
typedef struct { cs_ubx_frame_t frame; uint16_t pos; uint8_t state, a, b, received_a; uint32_t frames, checksum_errors, oversize; } cs_ubx_t;
typedef struct { uint32_t key; uint64_t value; } cs_ubx_kv_t;
void cs_ubx_init(cs_ubx_t *parser);
bool cs_ubx_feed(cs_ubx_t *parser, uint8_t byte);
bool cs_ubx_nav_pvt(const cs_ubx_frame_t *frame, uint32_t tick, uint8_t link_flags, cs_link_gnss_t *gnss);
/* Full UBX wire frame, 0 on invalid key size/layers/capacity. */
size_t cs_ubx_valset(const cs_ubx_kv_t *items, size_t count, uint8_t layers, uint8_t *out, size_t cap);
/* RAM configuration: 115200, UBX-only UART1, 10 Hz NAV-PVT, airborne <4g,
 * GPS+Galileo only. Key IDs verified in the interface description, not recalled.
 * Application sends at receiver's current baud then changes UART after ACK. */
size_t cs_ubx_flight_config(uint8_t *out, size_t cap);
#endif
