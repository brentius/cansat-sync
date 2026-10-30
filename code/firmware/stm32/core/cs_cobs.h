/* GPL-3.0-only. C twin of link.cobs_encode/decode, including 254-byte runs.
 * No delimiter; return false on malformed input/capacity failure. Empty input
 * decodes to empty; encode requires one byte. No overlapping input/output. */
#ifndef CS_COBS_H
#define CS_COBS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
bool cs_cobs_encode(const uint8_t *src, size_t len, uint8_t *dst, size_t cap, size_t *written);
bool cs_cobs_decode(const uint8_t *src, size_t len, uint8_t *dst, size_t cap, size_t *written);
#endif
