/* GPL-3.0-only. Protocol section 4: bounded nonblocking encoded-frame TX.
 * Single main-loop owner. peek pins the frame until consume: IMU may pass
 * every waiting frame but never interrupts the frame in progress. On a full
 * queue an IMU evicts the newest waiting non-IMU; otherwise drop the new one.
 * Each rejection/eviction increments tx_dropped. */
#ifndef CS_TXQUEUE_H
#define CS_TXQUEUE_H
#include "cs_link.h"
#define CS_TXQUEUE_CAPACITY 8u
typedef struct { uint8_t data[CS_LINK_MAX_WIRE]; size_t len; bool imu; } cs_txqueue_frame_t;
typedef struct {
    cs_txqueue_frame_t frames[CS_TXQUEUE_CAPACITY];
    size_t count, offset;
    bool active;
    uint32_t tx_dropped;
} cs_txqueue_t;
void cs_txqueue_init(cs_txqueue_t *queue);
bool cs_txqueue_push(cs_txqueue_t *queue, const uint8_t *wire, size_t len, bool imu);
const uint8_t *cs_txqueue_peek(cs_txqueue_t *queue, size_t *len);
bool cs_txqueue_consume(cs_txqueue_t *queue, size_t len);
#endif
