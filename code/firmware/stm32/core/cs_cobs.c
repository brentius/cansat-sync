/* GPL-3.0-only. C twin of link.cobs_encode/decode. */
#include "cs_cobs.h"

bool cs_cobs_encode(const uint8_t *src, size_t len, uint8_t *dst, size_t cap, size_t *written)
{
    size_t at = 0, out = 1, code_at = 0;
    unsigned code = 1;
    if (written) *written = 0;
    if (!written || !dst || (!src && len) || cap == 0) return false;
    while (at < len) {
        if (src[at] == 0) {
            dst[code_at] = (uint8_t)code;
            if (out >= cap) return false;
            code_at = out++;
            code = 1;
        } else {
            if (out >= cap) return false;
            dst[out++] = src[at];
            code++;
            if (code == 255) {
                dst[code_at] = 255;
                if (out >= cap) return false;
                code_at = out++;
                code = 1;
            }
        }
        at++;
    }
    dst[code_at] = (uint8_t)code;
    *written = out;
    return true;
}

bool cs_cobs_decode(const uint8_t *src, size_t len, uint8_t *dst, size_t cap, size_t *written)
{
    size_t at = 0, out = 0;
    if (written) *written = 0;
    if (!written || (!src && len) || (!dst && cap)) return false;
    while (at < len) {
        unsigned code = src[at++];
        size_t count = code ? (size_t)code - 1u : 0;
        if (!code || count > len - at || count > cap - out) return false;
        while (count--) dst[out++] = src[at++];
        if (code < 255 && at < len) {
            if (out >= cap) return false;
            dst[out++] = 0;
        }
    }
    *written = out;
    return true;
}
